package com.farmmachine.autosteer.can

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
 * AGMO Ver2(agmo_single) CAN 출력 브리지 — BnMcuCanService binder 직접 호출.
 *
 * 배경(ADB 실측): Ver2(Apollo2_10)엔 SocketCAN 인터페이스 없음. CAN = /dev/spidev2.0 →
 *   cpdevice MCU(cpcomm_server) → CAN 모터. binder `com.cpdevice.BnMcuCanService`(service #34) 중계.
 *
 * 설계(C안): Python BridgeBackend(TCP 13바이트 레코드 계약) **불변**. 이 브리지가 ApolloCanBridge 와
 *   동일한 TCP 서버 계약을 구현하되, 내부 전송을 VanMcu 대신 BnMcuCanService binder 로 한다.
 *   모터 프레임 ID(0x06000001 cmd_speed / 0x07000001 HB)는 Python CanSpec 그대로(중복정의 없음).
 *
 * 프로토콜(VER2_BINDER_PROTOCOL.md, libcpcomm.so 역분석 확정):
 *   - 디스크립터 = "com.cpdevice.BnMcuCanService"
 *   - TX  code 7  sendCanFrame: writeInterfaceToken + writeInt(N) + writeByteArray(N×14B)
 *   - RX  code 1  registerCallback: writeInterfaceToken + writeStrongBinder(우리 콜백)
 *   - 14바이트 프레임: [0]channel [1..3]canId(**big-endian**) [4](canId&0xFF)|flags
 *                      [5]dlc [6..13]data
 *   binder 호출은 public API(Parcel/IBinder.transact). ServiceManager 만 hidden → reflection.
 *
 * ⚠ 미확정(실차/추가분석 — TODO): ① byte[4] ext 플래그 비트 위치 ② 모터 CAN 채널 번호
 *   ③ RX 콜백 IBinder transact code/디스크립터 ④ Android11 hidden API 접근 제한(reflection 폴백).
 *   구조는 확정 — 위 4개는 안전 기본값 + TODO 로 두고 실차에서 최종 확인.
 */
class CpdeviceCanBridge(private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null
    private var binder: IBinder? = null
    private val rxQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(512)

    // TX drop 요약 로그(폭주 방지). channel/extFlag 는 companion(조정가능).
    @Volatile private var txDropped = 0
    @Volatile private var lastDropLogMs = 0L

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "CpdeviceCan"

    companion object {
        const val SERVICE_NAME = "com.cpdevice.BnMcuCanService"
        const val DESCRIPTOR = "com.cpdevice.BnMcuCanService"
        const val TX_SEND_CAN_FRAME = 7        // 확정(onTransact 점프테이블)
        const val TX_REGISTER_CALLBACK = 1     // 확정(registerCallback)
        // 확정(libcpcomm doSendProcess): MCU→콜백 transact code 19(0x13), flags=ONEWAY.
        //   Parcel = readInt32(count/ts, 무시) + createByteArray(14B × N).
        const val RX_TRANSACT_CODE = 19
        // ★ 미확정값 — UI 에서 바꿔가며 테스트(채널/ext 플래그). 기본 channel=0, extFlag=0x02.
        @Volatile var channel = 0      // byte[0] CAN 채널 (실차 0/1/2 스윕)
        @Volatile var extFlag = 0x02   // byte[4] 확장ID 플래그 비트(실차검증)

        // ★ 관찰 전용(observe-only) 기본 ON: binder 연결 + registerCallback(RX) 만. autokit2 세션을
        //   건드릴 수 있는 제어성 호출(setCANBaudrate / sendCanFrame TX) 일절 금지. RX 검증 단계 안전값.
        @Volatile var observeOnly = true
        // RX 콜백 등록 여부(공존 진단용): registerCallback 이 배타적이면 끄고 binder만 붙여 비교.
        @Volatile var registerRx = true

        // ★ TX 기본 비활성(안전): RX 검증을 먼저 한다(모터 자동 송신 금지). 채널/ext/RX 확정 후 수동 활성.
        @Volatile var txEnabled = false

        @Volatile var binderReady = false        // BnMcuCanService binder 획득
        @Volatile var clientConnected = false    // Python BridgeBackend 접속
        @Volatile var txCount = 0                // sendCanFrame transact 호출 수
        @Volatile var lastTxOk = false
        @Volatile var rxCount = 0                // 콜백 수신 프레임 수
        @Volatile var lastError = "init"
        @Volatile var instance: CpdeviceCanBridge? = null
    }

    fun start() {
        if (running) return
        running = true
        instance = this
        Log.i(TAG, "CpDev: bridge start (observeOnly=$observeOnly — no setCANBaudrate/TX, bus untouched)")
        // ★ binder 연결 + registerCallback 은 observe-only 와 무관하게 **항상** 실행(RX 수신 필수).
        try {
            connectBinder()
            registerRxCallback()
        } catch (e: Throwable) {
            Log.e(TAG, "CpDev ERR: init ${e.message}", e)
        }
        serverThread = thread(name = "cpdevice-can-bridge") { serve() }
        Log.i(TAG, "CpDev: bridge started :$port (binderReady=$binderReady)")
    }

    fun stop() {
        running = false
        clientConnected = false
        binder = null
        serverThread = null
    }

    fun setChannel(ch: Int) { channel = ch }   // 현장 진단용(채널 스윕)

    /** Acquire BnMcuCanService binder. ServiceManager is hidden API -> reflection. Never crash. */
    private fun connectBinder() {
        try {
            val sm = Class.forName("android.os.ServiceManager")
            val getService = sm.getMethod("getService", String::class.java)
            binder = getService.invoke(null, SERVICE_NAME) as? IBinder
            binderReady = (binder != null)
            // null -> service not registered OR Android11 hidden-API blocked OR SELinux denial.
            Log.i(TAG, "CpDev: getService($SERVICE_NAME) = %s %s".format(
                binder?.toString() ?: "null",
                if (binder == null) "(NULL -> not found / hidden-api blocked / selinux denial — grep 'avc: denied')" else ""))
            lastError = if (binderReady) "ok" else "getService null"
        } catch (e: Throwable) {
            binder = null; binderReady = false
            lastError = "ERR getService: ${e.message}"
            Log.e(TAG, "CpDev ERR: getService(reflection) ${e.message}", e)
        }
    }

    fun status(): String =
        """{"bridge":"cpdevice","binderReady":$binderReady,"connected":$clientConnected,""" +
        """"txCount":$txCount,"lastTxOk":$lastTxOk,"rxCount":$rxCount,"channel":$channel,""" +
        """"lastError":"${lastError.replace("\"","'")}"}"""

    // ── TX: 13바이트 TCP 레코드 → 14바이트 CAN 프레임 → sendCanFrame(code 7) ──────────
    private fun makeFrame14(canId: Int, dlc: Int, data: ByteArray): ByteArray {
        val ext = canId > 0x7FF                       // 29비트 확장 ID (Keya 0x06000001 등)
        val f = ByteArray(14)
        f[0] = channel.toByte()                       // ★ TODO(HW): 채널 실차확인
        f[1] = (canId ushr 24).toByte()               // canId big-endian (MSB first)
        f[2] = (canId ushr 16).toByte()
        f[3] = (canId ushr 8).toByte()
        f[4] = ((canId and 0xFF) or (if (ext) extFlag else 0)).toByte()  // ★ TODO: ext 비트 실차검증
        f[5] = dlc.coerceIn(0, 8).toByte()
        System.arraycopy(data, 0, f, 6, minOf(data.size, 8))
        return f
    }

    /**
     * 수동 단발 TX 테스트 — 버튼 1회 = 1프레임. observe-only/txEnabled 게이트 **우회**(명시적 사용자 동작).
     * 프레임 바이트(canId/data)는 Python CanSpec 에서 만들어 넘어온다(중복 정의 없음). 채널/ext 는 companion.
     * 보내기 직전 CpDev-TX hex 로그. binder 없으면 무송신.
     */
    fun txTestFrame(canId: Int, data: ByteArray): Boolean {
        val dlc = minOf(data.size, 8)
        Log.i("CpdeviceCan-TX", "CpDev-TX: ch=%d id=0x%08X dlc=%d data=%s".format(
            channel, canId, dlc, data.copyOf(dlc).joinToString("") { "%02X".format(it) }))
        val b = binder
        if (b == null) { Log.w("CpdeviceCan-TX", "CpDev-TX: binder=null (no send)"); return false }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        return try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeInt(1)
            p.writeByteArray(makeFrame14(canId, dlc, data))
            val ret = b.transact(TX_SEND_CAN_FRAME, p, r, 0)
            r.readException()
            lastTxOk = true; txCount++
            Log.i("CpdeviceCan-TX", "CpDev-TX: transact(code=$TX_SEND_CAN_FRAME) ret=$ret")
            true
        } catch (e: Throwable) {
            lastTxOk = false; lastError = "ERR txTest: ${e.message}"
            Log.e("CpdeviceCan-TX", "CpDev-TX ERR: transact ${e.message}", e)
            false
        } finally { p.recycle(); r.recycle() }
    }

    private fun sendCanFrame(canId: Int, dlc: Int, data: ByteArray) {
        // ★ 안전: observe-only(기본) 또는 TX 비활성이면 송신 금지(autokit2 버스 방해 방지).
        //   Python 이 고주파로 send 하므로 **조용히 drop**(매 호출 로그 금지). 1초당 1회만 요약 카운트.
        if (observeOnly || !txEnabled) {
            txDropped++
            val now = System.currentTimeMillis()
            if (now - lastDropLogMs >= 30000L) {   // 30s 요약(RX 로그가 묻히지 않게)
                Log.i(TAG, "CpDev: TX disabled - dropped %d (30s). cpdevTxEnable(true) after RX verify".format(txDropped))
                txDropped = 0; lastDropLogMs = now
            }
            return
        }
        val b = binder
        if (b == null) { lastTxOk = false; return }   // no binder -> silently skip (no guessed TX)
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeInt(1)                              // 프레임 개수 N=1
            p.writeByteArray(makeFrame14(canId, dlc, data))
            b.transact(TX_SEND_CAN_FRAME, p, r, 0)
            r.readException()
            lastTxOk = true; txCount++
            if (txCount % 50 == 1)
                Log.i(TAG, "TX sendCanFrame id=0x%08X dlc=%d ch=%d (#%d)".format(canId, dlc, channel, txCount))
        } catch (e: Throwable) {
            lastTxOk = false
            lastError = "TX transact failed: ${e.message}"
            Log.w(TAG, lastError)
        } finally {
            p.recycle(); r.recycle()
        }
    }

    // ── RX: registerCallback(code 1) — 우리 Binder 등록, 수신 14B 프레임을 rxQueue 로 ──────
    private val rxCallback = object : Binder() {
        override fun onTransact(code: Int, data: Parcel, reply: Parcel?, flags: Int): Boolean {
            // ★ 진입 즉시 무조건 1줄(code 무관). enforceInterface 호출 안 함(oneway라 토큰 없을 수 있음).
            //   확정: code 19(0x13) = readInt32(count/ts) + createByteArray(14B×N). 예외에도 안 죽게 try/catch.
            try {
                val dataSize = try { data.dataSize() } catch (_: Throwable) { -1 }
                Log.i("CpdeviceCan-RX", "CpDev-RX: code=%d size=%d flags=%d".format(code, dataSize, flags))
                rxCount++
                if (code == RX_TRANSACT_CODE) {
                    try {
                        val hdr = data.readInt()                   // count/timestamp — 무시 (Parcel.readInt)
                        val buf = data.createByteArray()
                        if (buf == null || buf.size < 14) {
                            Log.w("CpdeviceCan-RX", "code19 createByteArray=${buf?.size ?: -1}B (<14) -> raw fallback")
                            dumpRaw(data)
                            return true
                        }
                        val nFrames = buf.size / 14
                        Log.i("CpdeviceCan-RX", "code19 hdr=%d bytes=%d -> %d frames".format(hdr, buf.size, nFrames))
                        for (i in 0 until nFrames) {
                            val o = i * 14
                            val ch = buf[o].toInt() and 0xFF
                            val dlc = (buf[o + 5].toInt() and 0xFF).coerceIn(0, 8)
                            val low = (buf[o + 4].toInt() and 0xFF) and extFlag.inv()  // ext 플래그 제거(★TODO 실차)
                            val canId = ((buf[o + 1].toInt() and 0xFF) shl 24) or
                                        ((buf[o + 2].toInt() and 0xFF) shl 16) or
                                        ((buf[o + 3].toInt() and 0xFF) shl 8) or low
                            val payload = buf.copyOfRange(o + 6, o + 6 + dlc)
                            Log.i("CpdeviceCan-RX", "RX ch=%d id=0x%08X dlc=%d data=%s".format(
                                ch, canId, dlc, payload.joinToString("") { "%02X".format(it) }))
                            rxQueue.offer(canId to payload)     // push to existing TCP bridge RX record
                        }
                    } catch (e: Throwable) {
                        Log.w("CpdeviceCan-RX", "code19 parse failed: ${e.message} -> raw fallback")
                        dumpRaw(data)
                    }
                } else {
                    // non-19 code -> ignore (raw dump one line for analysis)
                    dumpRaw(data)
                }
            } catch (e: Throwable) {
                Log.w("CpdeviceCan-RX", "onTransact exception: ${e.message}")
            }
            return true
        }
    }

    /** Parcel raw hex 폴백(포맷 무관, 최대 64B). 마샬링이 다를 때 분석용. */
    private fun dumpRaw(data: Parcel) {
        try {
            val raw = data.marshall()
            val n = minOf(raw.size, 64)
            Log.i("CpdeviceCan-RX", "raw parcel %dB: %s%s".format(
                raw.size, (0 until n).joinToString(" ") { "%02X".format(raw[it]) }, if (raw.size > n) " …" else ""))
        } catch (e: Throwable) {
            Log.w("CpdeviceCan-RX", "marshall failed: ${e.message}")
        }
    }

    private fun registerRxCallback() {
        val b = binder
        Log.i(TAG, "CpDev: callback Binder created; registerCallback code=$TX_REGISTER_CALLBACK desc=\"$DESCRIPTOR\" binder=${b != null}")
        if (b == null) { Log.e(TAG, "CpDev ERR: registerCallback binder=null (getService failed)"); return }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeStrongBinder(rxCallback)
            Log.i(TAG, "CpDev: registerCallback transact(code=$TX_REGISTER_CALLBACK, writeStrongBinder) ...")
            val ret = b.transact(TX_REGISTER_CALLBACK, p, r, 0)
            r.readException()
            Log.i(TAG, "CpDev: registerCallback ret=$ret (true=delivered) — waiting RX code=$RX_TRANSACT_CODE")
            lastError = "registered(ret=$ret)"
        } catch (e: Throwable) {
            lastError = "ERR registerCallback: ${e.message}"
            Log.e(TAG, "CpDev ERR: registerCallback transact ${e.message}", e)   // graceful
        } finally {
            p.recycle(); r.recycle()
        }
    }

    // ── TCP 서버 (ApolloCanBridge 와 동일 13바이트 레코드 계약) ──────────
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

        // CAN → Python: 콜백 rxQueue 를 13B 레코드로 relay
        val rxThread = thread(name = "cpdevice-can-rx") {
            while (running && !sock.isClosed) {
                val f = try { rxQueue.poll(50, TimeUnit.MILLISECONDS) } catch (e: InterruptedException) { break } ?: continue
                try { synchronized(out) { out.write(encode(f.first.toLong() and 0xFFFFFFFFL, f.second.size, f.second)); out.flush() } }
                catch (e: Exception) { break }
            }
        }
        // keepalive (~1s) — Python rx_timeout 단선 감지용
        val hbThread = thread(name = "cpdevice-can-hb") {
            while (running && !sock.isClosed) {
                try { synchronized(out) { out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush() }; Thread.sleep(1000) }
                catch (e: Exception) { break }
            }
        }
        // Python → CAN: 13B 레코드 파싱 → sendCanFrame
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
