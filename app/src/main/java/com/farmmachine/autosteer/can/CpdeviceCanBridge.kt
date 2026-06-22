package com.farmmachine.autosteer.can

import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.os.Binder
import android.os.IBinder
import android.os.Parcel
import android.util.Log
import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * AGMO Ver2(agmo_single, Apollo2_10) CAN 브리지 — **원본 앱 실측 프로토콜** 자체 구현.
 *
 * ★ 정정(app-decompiled 역분석 확정): 원본은 `com.cpdevice.BnMcuCanService` 에 raw transact(code3/7/1)
 *   를 쓰지 **않는다**. 다음 경로를 쓴다(IRemoteService/CommunicationService/Command/Spring 확인):
 *   - **AIDL 바인드 서비스**: bindService(action="com.android.guard.E9631Service", pkg="com.android.guard")
 *     · 디스크립터 "com.android.guard.IRemoteService"
 *     · handleData(byte[]) = txn 1 (송신) / registerCallback(cb) = txn 2 / RX = IRemoteServiceCallBack.valueChanged(byte[]) txn1
 *   - **프레이밍**(Command.createProtocolPacket): 내부명령 [type][len_hi][len_lo][payload]
 *     → BCC 2바이트(b+=x; b2+=b) → 0x55→0x55 0x55 이스케이프 → START(55 02)…ENDOF(55 03)
 *   - **CAN 송신**: sendData(payload, type=0x40), payload = [0x00][canId 4B BE][dataLen][data…]
 *   - **CAN 개통**: 모드 0x80 00 01 03(CAN) → 속도 0x30 00 01 {0x12=125k,0x25=250k,0x50=500k} → 채널 0x82 00 01 {0/1/2}
 *   - **RX**: valueChanged 로 디프레이밍된 [type][len_hi][len_lo][payload]; type 0x41 = 수신 CAN.
 *
 * 설계: Python BridgeBackend(TCP 13바이트 레코드 계약) **불변** — ApolloCanBridge 와 동일 TCP 서버,
 *   내부 전송만 위 프레이밍으로. 모터 프레임 ID(0x06000001 cmd_speed / 0x07000001 HB)는 Python CanSpec 그대로.
 *
 * ⚠ 미확정(실차): ① CAN payload 선두 byte0(원본 고정 0x00; 채널은 0x82 명령) ② RX CAN payload 정확 레이아웃
 *   (송신 미러로 가정: [00][canId4][dlc][data]) ③ com.android.guard 서비스 export/가시성(<queries> 추가).
 */
class CpdeviceCanBridge(private val context: Context, private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null
    @Volatile private var service: IBinder? = null     // IRemoteService 원격 핸들
    @Volatile private var bound = false
    private val rxQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(512)

    // TX drop 요약 로그(폭주 방지).
    @Volatile private var txDropped = 0
    @Volatile private var lastDropLogMs = 0L

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "CpdeviceCan"

    companion object {
        // ── com.android.guard IRemoteService (원본 앱 확정) ──
        const val SVC_ACTION = "com.android.guard.E9631Service"
        const val SVC_PKG = "com.android.guard"
        const val IFACE = "com.android.guard.IRemoteService"
        const val CB_IFACE = "com.android.guard.IRemoteServiceCallBack"
        const val TXN_HANDLE_DATA = 1
        const val TXN_REGISTER_CALLBACK = 2
        const val TXN_VALUE_CHANGED = 1

        // ── 프레이밍 상수 ──
        const val ESC = 0x55; const val STX = 0x02; const val ETX = 0x03
        // 명령 type(내부명령 [0])
        const val TYPE_CAN = 0x40        // 송신 CAN
        const val TYPE_SWITCH = 0x30     // baud: 0x12=125k 0x25=250k 0x50=500k
        const val TYPE_MODE = 0x80       // 0x03 = CAN 모드
        const val TYPE_CHANNEL = 0x82    // 0/1/2
        const val RX_TYPE_CAN = 0x41     // 수신 CAN(valueChanged)

        // ★ CAN 채널/속도 — 우선 ch=0, 250k. 무반응 시 ch/baud 스윕.
        @Volatile var channel = 0
        @Volatile var baud = 250000
        @Volatile var baud2 = 250000     // (호환 유지용; 프레이밍 프로토콜은 단일 baud 코드 사용)

        @Volatile var canOpened = false  // 모드/baud/채널 개통 완료
        @Volatile var observeOnly = true // 고주파 자동 스트리밍 TX 차단(수동 버튼/개통은 허용)
        @Volatile var registerRx = true  // 정식 AIDL 콜백 → 안전(이전 raw transact code1 크래시 해소)
        @Volatile var txEnabled = false  // Python 고주파 스트리밍 게이트

        @Volatile var binderReady = false
        @Volatile var clientConnected = false
        @Volatile var txCount = 0
        @Volatile var lastTxOk = false
        @Volatile var rxCount = 0
        @Volatile var lastError = "init"
        @Volatile var instance: CpdeviceCanBridge? = null
    }

    // ── 바인드 ──────────────────────────────────────────────
    private val conn = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, binder: IBinder) {
            service = binder
            binderReady = true
            lastError = "bound"
            Log.i(TAG, "CpDev: bound $IFACE ($name)")
            try {
                if (registerRx) registerCallbackRemote()   // RX 콜백(정식 AIDL)
                openCan()                                   // 모드 CAN + baud + 채널
            } catch (e: Throwable) {
                lastError = "post-bind init: ${e.message}"
                Log.e(TAG, "CpDev: post-bind init err", e)
            }
        }
        override fun onServiceDisconnected(name: ComponentName?) {
            service = null; binderReady = false; canOpened = false
            lastError = "service disconnected"
            Log.w(TAG, "CpDev: service disconnected -> $name")
        }
    }

    private fun bindGuard() {
        val intent = Intent(SVC_ACTION).apply { setPackage(SVC_PKG) }
        bound = try {
            context.bindService(intent, conn, Context.BIND_AUTO_CREATE)
        } catch (e: Throwable) {
            lastError = "bindService ex: ${e.message}"; Log.e(TAG, "CpDev: bindService ex", e); false
        }
        Log.i(TAG, "CpDev: bindService($SVC_ACTION pkg=$SVC_PKG) = $bound")
        if (!bound) lastError = "bindService=false (서비스 미설치/미export 또는 <queries> 가시성)"
    }

    fun start() {
        if (running) return
        running = true
        instance = this
        Log.i(TAG, "CpDev: bridge start (com.android.guard IRemoteService + 프레이밍)")
        bindGuard()
        serverThread = thread(name = "cpdevice-can-bridge") { serve() }
        Log.i(TAG, "CpDev: bridge started :$port")
    }

    fun stop() {
        running = false
        clientConnected = false
        try { if (bound) context.unbindService(conn) } catch (_: Throwable) {}
        bound = false; service = null; binderReady = false; canOpened = false
        serverThread = null
    }

    fun setChannel(ch: Int) { channel = ch }

    // ── RX 콜백(우리 Binder) — guard 서비스가 valueChanged(txn1) 로 역호출 ──
    private val rxCallback = object : Binder() {
        override fun onTransact(code: Int, data: Parcel, reply: Parcel?, flags: Int): Boolean {
            if (code == TXN_VALUE_CHANGED) {
                try {
                    data.enforceInterface(CB_IFACE)
                    val payload = data.createByteArray()
                    reply?.writeNoException()
                    if (payload != null) handleRx(payload)
                } catch (e: Throwable) {
                    Log.w("CpdeviceCan-RX", "valueChanged 처리 예외: ${e.message}")
                }
                return true
            }
            if (code == 1598968902) { reply?.writeString(CB_IFACE); return true }  // INTERFACE_TRANSACTION
            return super.onTransact(code, data, reply, flags)
        }
    }

    /** registerCallback(IRemoteServiceCallBack) = IRemoteService txn 2. */
    private fun registerCallbackRemote() {
        val b = service ?: run { lastError = "registerCallback: not bound"; return }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(IFACE)
            p.writeStrongBinder(rxCallback)
            b.transact(TXN_REGISTER_CALLBACK, p, r, 0)
            r.readException()
            lastError = "registered"
            Log.i(TAG, "CpDev: registerCallback ok (RX valueChanged 대기)")
        } catch (e: Throwable) {
            lastError = "registerCallback: ${e.message}"; Log.e(TAG, "CpDev: registerCallback err", e)
        } finally { p.recycle(); r.recycle() }
    }

    fun registerRxNow(): String { registerRx = true; registerCallbackRemote(); return lastError }

    // ── 송신: handleData(framed) = IRemoteService txn 1 ──
    private fun handleData(frame: ByteArray): Boolean {
        val b = service ?: run { lastError = "handleData: not bound"; return false }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        return try {
            p.writeInterfaceToken(IFACE)
            p.writeByteArray(frame)
            b.transact(TXN_HANDLE_DATA, p, r, 0)
            r.readException()
            true
        } catch (e: Throwable) {
            lastTxOk = false; lastError = "handleData: ${e.message}"
            Log.e(TAG, "CpDev: handleData err", e); false
        } finally { p.recycle(); r.recycle() }
    }

    // ── 프레이밍: 내부명령 → BCC → escape → STX..ETX (Command.createProtocolPacket 자체구현) ──
    private fun frameOf(inner: ByteArray): ByteArray {
        var b = 0; var b2 = 0
        for (x in inner) { b = (b + (x.toInt() and 0xFF)) and 0xFF; b2 = (b2 + b) and 0xFF }
        val withBcc = ByteArray(inner.size + 2)
        System.arraycopy(inner, 0, withBcc, 0, inner.size)
        withBcc[inner.size] = b.toByte()
        withBcc[inner.size + 1] = b2.toByte()
        val esc = ArrayList<Byte>(withBcc.size + 8)
        for (x in withBcc) { esc.add(x); if ((x.toInt() and 0xFF) == ESC) esc.add(ESC.toByte()) }
        val out = ByteArray(esc.size + 4)
        out[0] = ESC.toByte(); out[1] = STX.toByte()
        for (i in esc.indices) out[i + 2] = esc[i]
        out[out.size - 2] = ESC.toByte(); out[out.size - 1] = ETX.toByte()
        return out
    }

    /** 내부명령 [type][len_hi][len_lo][payload] (Command.Send.sendData). */
    private fun sendData(type: Int, payload: ByteArray): Boolean {
        val inner = ByteArray(payload.size + 3)
        inner[0] = type.toByte()
        inner[1] = ((payload.size ushr 8) and 0xFF).toByte()
        inner[2] = (payload.size and 0xFF).toByte()
        System.arraycopy(payload, 0, inner, 3, payload.size)
        return handleData(frameOf(inner))
    }

    /** type/len/payload 가 이미 들어있는 짧은 내부명령(switch/mode/channel). */
    private fun sendInner(vararg bytes: Int): Boolean {
        val inner = ByteArray(bytes.size)
        for (i in bytes.indices) inner[i] = bytes[i].toByte()
        return handleData(frameOf(inner))
    }

    /** 모드 CAN → baud → 채널 순서로 개통. */
    fun openCan(): Boolean {
        val sw = when (baud) { 125000 -> 0x12; 500000 -> 0x50; else -> 0x25 }   // 기본 250k
        val ok = sendInner(TYPE_MODE, 0x00, 0x01, 0x03) &&                       // CAN 모드
                 sendInner(TYPE_SWITCH, 0x00, 0x01, sw) &&                        // baud
                 sendInner(TYPE_CHANNEL, 0x00, 0x01, channel and 0xFF)           // 채널
        canOpened = ok
        lastError = if (ok) "canOpened(baud=$baud ch=$channel)" else "openCan fail"
        Log.i(TAG, "CpDev: openCan mode=CAN baud=%d(sw=0x%02X) ch=%d ok=%b".format(baud, sw, channel, ok))
        return ok
    }

    /**
     * CAN 프레임 송신. payload = [0x00][canId 4B BE][dataLen][data…] (원본 byteArrayAddByteArray) → type 0x40.
     * canId/data 는 Python CanSpec 에서 만들어 넘어옴(중복 정의 없음).
     */
    private fun sendCan(canId: Int, data: ByteArray): Boolean {
        val dlc = minOf(data.size, 8)
        val payload = ByteArray(6 + dlc)
        payload[0] = 0x00                                  // 원본 고정 0(채널은 0x82 별도). ★ 실차검증
        payload[1] = ((canId ushr 24) and 0xFF).toByte()   // canId big-endian
        payload[2] = ((canId ushr 16) and 0xFF).toByte()
        payload[3] = ((canId ushr 8) and 0xFF).toByte()
        payload[4] = (canId and 0xFF).toByte()
        payload[5] = dlc.toByte()
        System.arraycopy(data, 0, payload, 6, dlc)
        Log.i("CpdeviceCan-TX", "CpDev-TX: id=0x%08X dlc=%d data=%s frame=%s".format(
            canId, dlc, data.copyOf(dlc).joinToString("") { "%02X".format(it) },
            payload.joinToString("") { "%02X".format(it) }))
        val ok = sendData(TYPE_CAN, payload)
        if (ok) { txCount++; lastTxOk = true } else lastTxOk = false
        return ok
    }

    /** 수동 단발 TX(버튼) — observe-only/txEnabled 게이트 우회. CAN 미개통이면 먼저 개통. */
    fun txTestFrame(canId: Int, data: ByteArray): Boolean {
        if (!canOpened) openCan()
        return sendCan(canId, data)
    }

    @Volatile private var burstStop = false

    /** 버스트 TX: enable 1회 → ms 동안 50ms 마다 cmd 재전송(워치독 회피) → disable. 잭업 검증용. */
    fun txBurst(cmdId: Int, cmdData: ByteArray, enableData: ByteArray, disableData: ByteArray, ms: Int) {
        burstStop = true
        thread(name = "cpdev-burst") {
            burstStop = false
            try {
                Log.i("CpdeviceCan-TX", "CpDev-TX: BURST start ch=$channel ${ms}ms")
                txTestFrame(cmdId, enableData)
                val end = System.currentTimeMillis() + ms.toLong()
                while (!burstStop && System.currentTimeMillis() < end) {
                    txTestFrame(cmdId, cmdData); Thread.sleep(50)
                }
            } catch (e: Throwable) {
                Log.w("CpdeviceCan-TX", "CpDev-TX: BURST err ${e.message}")
            } finally {
                try { txTestFrame(cmdId, disableData) } catch (_: Throwable) {}
                Log.i("CpdeviceCan-TX", "CpDev-TX: BURST end (disable sent)")
            }
        }
    }

    fun stopBurst() { burstStop = true }

    @Volatile private var sweeping = false

    /** 채널 스윕(0/1/2) — 어느 채널에서 모터 HB(RX) 가 올라오는지 확인용. */
    fun openCanSweep(baud: Int = 250000): String {
        if (sweeping) return "already-sweeping"
        sweeping = true
        thread(name = "cpdev-sweep") {
            try {
                Companion.baud = baud
                for (ch in 0..2) {
                    channel = ch
                    val before = rxCount
                    openCan()
                    Thread.sleep(800)                     // HB 올라올 시간
                    Log.i(TAG, "CpDev-SWEEP: ch=%d rxDelta=%d".format(ch, rxCount - before))
                }
                Log.i(TAG, "CpDev-SWEEP: done rxCount=$rxCount (RX 올라온 채널 = 모터 채널)")
            } catch (e: Throwable) {
                Log.e(TAG, "CpDev-SWEEP err: ${e.message}", e)
            } finally { sweeping = false }
        }
        return "sweep-started"
    }

    fun status(): String =
        """{"bridge":"cpdevice","binderReady":$binderReady,"canOpened":$canOpened,"connected":$clientConnected,""" +
        """"txCount":$txCount,"lastTxOk":$lastTxOk,"rxCount":$rxCount,"channel":$channel,"baud":$baud,""" +
        """"lastError":"${lastError.replace("\"", "'")}"}"""

    // ── RX: valueChanged 디프레이밍 페이로드 [type][len_hi][len_lo][payload] → rxQueue ──
    private fun handleRx(inner: ByteArray) {
        rxCount++
        val type = inner[0].toInt() and 0xFF
        val len = if (inner.size >= 3) (((inner[1].toInt() and 0xFF) shl 8) or (inner[2].toInt() and 0xFF)) else 0
        Log.i("CpdeviceCan-RX", "CpDev-RX: type=0x%02X len=%d total=%d".format(type, len, inner.size))
        // 수신 CAN: payload(송신 미러 가정) = [00][canId 4B][dlc][data]. ★ 실차 레이아웃 확인.
        if (type == RX_TYPE_CAN && inner.size >= 3 + 6) {
            val o = 3
            val canId = ((inner[o + 1].toInt() and 0xFF) shl 24) or
                        ((inner[o + 2].toInt() and 0xFF) shl 16) or
                        ((inner[o + 3].toInt() and 0xFF) shl 8) or
                         (inner[o + 4].toInt() and 0xFF)
            val dlc = (inner[o + 5].toInt() and 0xFF).coerceIn(0, 8)
            val end = minOf(o + 6 + dlc, inner.size)
            val payload = if (end > o + 6) inner.copyOfRange(o + 6, end) else ByteArray(0)
            Log.i("CpdeviceCan-RX", "RX id=0x%08X dlc=%d data=%s".format(
                canId, dlc, payload.joinToString("") { "%02X".format(it) }))
            rxQueue.offer(canId to payload)
        }
    }

    // ── TCP 서버 (ApolloCanBridge 와 동일 13바이트 레코드 계약) ──
    private fun sendCanFrame(canId: Int, dlc: Int, data: ByteArray) {
        // 안전: observe-only(기본) 또는 TX 비활성이면 조용히 drop(30s 요약 카운트).
        if (observeOnly || !txEnabled) {
            txDropped++
            val now = System.currentTimeMillis()
            if (now - lastDropLogMs >= 30000L) {
                Log.i(TAG, "CpDev: TX disabled - dropped %d (30s). cpdevTxEnable(true) after RX verify".format(txDropped))
                txDropped = 0; lastDropLogMs = now
            }
            return
        }
        if (!canOpened) openCan()
        sendCan(canId, data)
    }

    private fun serve() {
        val server = ServerSocket(port).apply { reuseAddress = true; soTimeout = 200 }
        while (running) {
            val sock = try { server.accept() } catch (e: Exception) { continue }
            try { handle(sock) } catch (e: Exception) { Log.w(TAG, "client closed: ${e.message}") }
            try { sock.close() } catch (_: Exception) {}
        }
        try { server.close() } catch (_: Exception) {}
    }

    private fun handle(sock: Socket) {
        sock.tcpNoDelay = true
        clientConnected = true
        val inp = DataInputStream(sock.getInputStream())
        val out = sock.getOutputStream()

        val rxThread = thread(name = "cpdevice-can-rx") {
            while (running && !sock.isClosed) {
                val f = try { rxQueue.poll(50, TimeUnit.MILLISECONDS) } catch (e: InterruptedException) { break } ?: continue
                try { synchronized(out) { out.write(encode(f.first.toLong() and 0xFFFFFFFFL, f.second.size, f.second)); out.flush() } }
                catch (e: Exception) { break }
            }
        }
        val hbThread = thread(name = "cpdevice-can-hb") {
            while (running && !sock.isClosed) {
                try { synchronized(out) { out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush() }; Thread.sleep(1000) }
                catch (e: Exception) { break }
            }
        }
        val rec = ByteArray(REC)
        try {
            while (running) {
                inp.readFully(rec)
                val id = (((rec[0].toInt() and 0xFF) shl 24) or
                          ((rec[1].toInt() and 0xFF) shl 16) or
                          ((rec[2].toInt() and 0xFF) shl 8) or
                           (rec[3].toInt() and 0xFF))
                val dlc = (rec[4].toInt() and 0xFF).coerceIn(0, 8)
                val data = rec.copyOfRange(5, 5 + dlc)
                if ((id.toLong() and 0xFFFFFFFFL) != HEARTBEAT) sendCanFrame(id, dlc, data)
            }
        } finally {
            clientConnected = false
            rxThread.interrupt(); hbThread.interrupt()
        }
    }

    private fun encode(id: Long, dlc: Int, data: ByteArray): ByteArray {
        val b = ByteArray(REC)
        b[0] = (id ushr 24).toByte(); b[1] = (id ushr 16).toByte()
        b[2] = (id ushr 8).toByte(); b[3] = id.toByte()
        b[4] = dlc.toByte()
        System.arraycopy(data, 0, b, 5, minOf(data.size, 8))
        return b
    }
}
