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
 *   - 14바이트 프레임(makeCanSendBuffer 역분석 확정): [0]channel
 *       [1..4] ID워드 빅엔디안 — ext: word=((id<<3)|0x04) / std: word=(id<<21)
 *       [5]dlc [6..13]data.  검산: 0x06000001→30 00 00 0C, 0x07000001→38 00 00 0C.
 *   binder 호출은 public API(Parcel/IBinder.transact). ServiceManager 만 hidden → reflection.
 *
 * ⚠ 미확정(실차/추가분석 — TODO): ① byte[4] ext 플래그 비트 위치 ② 모터 CAN 채널 번호
 *   ③ RX 콜백 IBinder transact code/디스크립터 ④ Android11 hidden API 접근 제한(reflection 폴백).
 *   구조는 확정 — 위 4개는 안전 기본값 + TODO 로 두고 실차에서 최종 확인.
 */
class CpdeviceCanBridge(private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null
    private var binder: IBinder? = null               // 단일 서비스 핸들(register/TX 공용)
    private val rxQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(512)

    // binder 사망 감지 → 자동 재연결
    private val deathRecipient = IBinder.DeathRecipient {
        binderReady = false; binder = null
        lastError = "binder died"
        Log.w(TAG, "CpDev: binder DIED -> will reconnect on next use")
    }

    // TX drop 요약 로그(폭주 방지). channel 은 companion(조정가능).
    @Volatile private var txDropped = 0
    @Volatile private var lastDropLogMs = 0L

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "CpdeviceCan"

    companion object {
        const val SERVICE_NAME = "com.cpdevice.BnMcuCanService"
        const val DESCRIPTOR = "com.cpdevice.BnMcuCanService"
        // ★ 역분석 확정(libcpcomm.so Bp 메서드 디스어셈블) — BnMcuCanService 트랜잭션 코드 맵.
        //   이전 추측(code1=getMcuVersion, registerCallback=16, RX=19)은 전부 틀렸음(아래가 확정).
        const val CODE_REGISTER_CALLBACK = 1     // registerCallback(IBinder)
        const val CODE_UNREGISTER_CALLBACK = 2   // unregisterCallback
        const val CODE_SET_CAN_BAUDRATE = 3      // setCANBaudrate(int ch, uint baud, uint baud2) — CAN 열기 ★
        const val CODE_SEND_CAN_FRAME = 7        // sendCanFrame(int N, byte[N*14])
        const val CODE_GET_MCU_VERSION = 9       // getMcuVersion
        // RX 콜백: 등록한 IBinder 로 서비스가 **역transact**. 그 code 는 콜백쪽 구현(미고정)이라
        //   onTransact 에서 고정 code 가정 없이 프레임형 페이로드를 파싱한다.

        // ★ CAN 채널/속도 — 우선 ch=0, baud=250000. 무반응 시 ch=1, baud=500000 등으로 바꿔 테스트.
        @Volatile var channel = 0
        @Volatile var baud = 250000
        @Volatile var baud2 = 250000   // 2번째 baud 인자(CAN-FD data baud 추정) — 우선 baud 동일값

        @Volatile var canOpened = false  // setCANBaudrate(code3) 성공 → 이후에야 sendCanFrame 유효

        // observe-only: 고주파 자동 스트리밍 TX 만 차단(개통/RX 등록/수동 버튼 TX 는 허용).
        @Volatile var observeOnly = true
        // ★ RX(code1) 자동 등록 끔: registerCallback(code1) transact 가 cpcomm_server 를 죽여(매번 DeadObject)
        //   TX(code7)까지 막던 무한루프 원인(logcat3 22:23 실측). code1 역transact IF 확정 전엔 수동/진단만 등록.
        @Volatile var registerRx = false
        // ★ Python 고주파 스트리밍 게이트(안전). 수동 점검 버튼(txTestFrame)은 우회.
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
        Log.i(TAG, "CpDev: bridge start — 확정 순서: getService -> setCANBaudrate(code3) -> registerCallback(code1)")
        try {
            connectBinder()
            // ★ 개통→재시작대기→콜백등록은 블로킹(최대 ~3s)이라 백그라운드 스레드로(UI/JsBridge 비블로킹).
            thread(name = "cpdev-init") { initCan() }
        } catch (e: Throwable) {
            Log.e(TAG, "CpDev ERR: init ${e.message}", e)
        }
        serverThread = thread(name = "cpdevice-can-bridge") { serve() }
        Log.i(TAG, "CpDev: bridge started :$port (init in background)")
    }

    /**
     * init 순서(logcat 실측 확정): setCANBaudrate(code3) 로 개통만. 단독 code3 는 서비스를 죽이지 않음
     *   (logcat41 22:39: code3 후 DIED 없음). 서비스를 죽이던 건 registerCallback(code1) 였음(logcat3 22:23).
     *   → RX(code1) 자동 등록 생략. 모터 제어는 sendCanFrame(code7) 만으로 충분(code7 ret=true 실측).
     */
    private fun initCan() {
        openCan()                                   // code3 — CAN 개통(canOpened=true). 단독으론 서비스 안 죽음.
        Log.i(TAG, "CpDev: RX 콜백 등록 생략(서비스 크래시 방지). 모터 제어=TX(code7)만. RX 는 code1 IF 확정 후.")
        Log.i(TAG, "CpDev: initCan done binderReady=$binderReady canOpened=$canOpened")
    }

    /**
     * setCANBaudrate(code 3, ch, baud, baud2) — CAN 채널 열기. **모든 TX 전에 1회 필수.**
     * 마샬링(확정): writeInterfaceToken + writeInt(ch) + writeInt(baud) + writeInt(baud2), transact(3, …, flags=1 oneway).
     * 성공 시 canOpened=true. 미개통 상태에서 sendCanFrame(code7) 보내면 서비스가 핸들 무효화(DeadObject)였음.
     */
    fun openCan(): Boolean {
        val b = ensureBinder()
        if (b == null) { canOpened = false; lastError = "openCan: binder null"; Log.e(TAG, "CpDev: openCan binder=null"); return false }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        return try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeInt(channel)
            p.writeInt(baud)
            p.writeInt(baud2)
            val ret = b.transact(CODE_SET_CAN_BAUDRATE, p, r, 1)   // flags=1 ONEWAY
            canOpened = true
            lastError = "canOpened(ch=$channel baud=$baud)"
            Log.i(TAG, "CpDev: setCANBaudrate(code3) ch=%d baud=%d baud2=%d ret=%b -> canOpened=true".format(channel, baud, baud2, ret))
            true
        } catch (e: Throwable) {
            canOpened = false; binder = null
            lastError = "openCan ERR: ${e.message}"
            Log.e(TAG, "CpDev: setCANBaudrate(code3) FAIL ${e.message}", e)
            false
        } finally { p.recycle(); r.recycle() }
    }

    fun stop() {
        running = false
        clientConnected = false
        binder = null
        serverThread = null
    }

    fun setChannel(ch: Int) { channel = ch }   // 현장 진단용(채널 스윕)

    /** 수동 RX 콜백 등록(code 16) — TX 와 분리. 서비스 죽으면 다음 TX 가 재연결로 복구. */
    fun registerRxNow(): String { registerRx = true; ensureBinder(); registerRxCallback(); return lastError }

    @Volatile private var burstStop = false

    /** 버스트 TX: enable 1회 → ms 동안 50ms 마다 cmdData 재전송(Keya 1s 워치독 회피) → 정지(disable).
     *  잭업 검증용. 채널 바꿔가며 어느 채널에서 모터가 도는지 확인. estop/재호출 시 중단. */
    fun txBurst(cmdId: Int, cmdData: ByteArray, enableData: ByteArray, disableData: ByteArray, ms: Int) {
        burstStop = true                       // 기존 버스트 중단
        thread(name = "cpdev-burst") {
            burstStop = false
            try {
                Log.i("CpdeviceCan-TX", "CpDev-TX: BURST start ch=$channel ${ms}ms")
                txTestFrame(cmdId, enableData)
                val end = System.currentTimeMillis() + ms.toLong()
                while (!burstStop && System.currentTimeMillis() < end) {
                    txTestFrame(cmdId, cmdData)
                    Thread.sleep(50)
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

    /** 서비스가 죽었으면 init 가 재시작할 때까지 대기(최대 maxMs). 살아있는 binder 반환 or null. */
    private fun waitForService(maxMs: Long): IBinder? {
        val end = System.currentTimeMillis() + maxMs
        while (System.currentTimeMillis() < end) {
            val b = binder
            if (b != null && (try { b.isBinderAlive } catch (_: Throwable) { false })) return b
            connectBinder()                                   // getService 재시도(재시작 후 핸들 재획득)
            if (binderReady) return binder
            try { Thread.sleep(300) } catch (_: Throwable) {}  // 재시작 시간 양보(폭주 금지)
        }
        return binder
    }

    /**
     * CAN 개통(setCANBaudrate/openCan) transact code 탐색 — 실차 진단(하드닝판).
     * 배경: registerCallback(code16) 후에도 RX(code19) 0건 → CAN 미개통 추정. setCANBaudrate code 미확정.
     * 실측 1차 스윕: code 2/3 은 실재(ret=true)하나 호출 시 native 서비스(cpcomm_server)가 죽음.
     *   → 이 판은 (a) 죽으면 재시작까지 **대기**(최대 5s, 폭주 금지) (b) **어느 code 가 죽이는지** 명시
     *   (c) 재시작 실패가 길면 **중단**(무한 null 스팸 방지).
     * 판정: RX(모터 HB 0x07000001)가 살아나는 code = 개통 code. 어느 code 에도 RX=0 이면
     *   blind 탐색 한계 → 역분석(setCANBaudrate 시그니처) 필요.
     */
    fun openCanSweep(baud: Int = 250000): String {
        if (sweeping) return "already-sweeping"
        sweeping = true
        thread(name = "cpdev-sweep") {
            var deaths = 0
            val crashCodes = ArrayList<String>()
            try {
                if (!registerRx) { registerRx = true; registerRxCallback() }
                Log.i(TAG, "CpDev-SWEEP: start baud=$baud ch=$channel (RX registered, watching rxCount)")
                val skip = setOf(1, 7, 16, 19)         // 1=getMcuVersion 7=sendCanFrame 16=registerCallback 19=RX
                loop@ for (code in 2..24) {
                    if (code in skip) continue
                    for (shape in 0..1) {
                        // 서비스 살아날 때까지 대기. 끝내 null 이면 init 가 못 살림 → 중단.
                        val b = waitForService(5000)
                        if (b == null) {
                            Log.w(TAG, "CpDev-SWEEP: code=$code service down >5s -> ABORT (재시작 실패)")
                            break@loop
                        }
                        val before = rxCount
                        val p = Parcel.obtain(); val r = Parcel.obtain()
                        var ret = false; var err = "-"
                        try {
                            p.writeInterfaceToken(DESCRIPTOR)
                            if (shape == 0) { p.writeInt(channel); p.writeInt(baud) }  // setCANBaudrate(int ch,int baud)
                            else { p.writeInt(baud) }                                  // setCANBaudrate(int baud)
                            ret = b.transact(code, p, r, 0)
                            try { r.readException() } catch (e: Throwable) { err = "exc:${e.message}" }
                        } catch (e: android.os.DeadObjectException) {
                            err = "DeadObject"; binder = null
                        } catch (e: Throwable) {
                            err = e.javaClass.simpleName + ":" + e.message; binder = null
                        } finally { p.recycle(); r.recycle() }
                        Thread.sleep(600)                  // 개통되면 이 사이에 HB 가 올라옴
                        val rxDelta = rxCount - before
                        val died = !(try { binder?.isBinderAlive ?: false } catch (_: Throwable) { false })
                        if (died) { deaths++; crashCodes.add("$code/s$shape") }
                        Log.i(TAG, "CpDev-SWEEP: code=%d shape=%d ret=%b err=%s rxDelta=%d died=%b".format(
                            code, shape, ret, err, rxDelta, died))
                        if (rxDelta > 0) Log.i(TAG, "CpDev-SWEEP: *** code=$code shape=$shape -> RX ALIVE (likely CAN-open code) ***")
                        if (died) Log.w(TAG, "CpDev-SWEEP: !!! code=$code shape=$shape -> service DIED (이 code 가 서비스를 죽임) !!!")
                    }
                }
                Log.i(TAG, "CpDev-SWEEP: done. rxCount=%d deaths=%d crashCodes=%s".format(
                    rxCount, deaths, crashCodes.joinToString(",")))
                if (rxCount == 0) Log.w(TAG, "CpDev-SWEEP: RX 0건 — blind 탐색 한계. 역분석 setCANBaudrate 시그니처 필요.")
            } catch (e: Throwable) {
                Log.e(TAG, "CpDev-SWEEP ERR: ${e.message}", e)
            } finally { sweeping = false }
        }
        return "sweep-started"
    }

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
            try { binder?.linkToDeath(deathRecipient, 0) } catch (e: Throwable) { Log.w(TAG, "linkToDeath fail: ${e.message}") }
        } catch (e: Throwable) {
            binder = null; binderReady = false
            lastError = "ERR getService: ${e.message}"
            Log.e(TAG, "CpDev ERR: getService(reflection) ${e.message}", e)
        }
    }

    /** transact 전 호출: binder 살아있으면 그대로, 죽었/없으면 getService 재획득.
     *  ★ RX registerCallback 은 **여기서 부르지 않는다** — code1 등록이 서비스를 죽여 TX 가
     *  매번 실패하던 무한 루프 원인이었음. RX 등록은 별도 수동(tryRegisterRx)으로만. */
    private fun ensureBinder(): IBinder? {
        val b = binder
        if (b != null && (try { b.isBinderAlive } catch (_: Throwable) { false })) return b
        Log.w(TAG, "CpDev: binder dead/null -> reconnect (getService again)")
        connectBinder()
        // ★ RX 콜백 재등록 안 함: registerCallback(code1) 이 cpcomm_server 를 죽여 TX 가 매번 실패하던 원인(logcat3).
        return binder
    }

    fun status(): String =
        """{"bridge":"cpdevice","binderReady":$binderReady,"connected":$clientConnected,""" +
        """"txCount":$txCount,"lastTxOk":$lastTxOk,"rxCount":$rxCount,"channel":$channel,""" +
        """"lastError":"${lastError.replace("\"","'")}"}"""

    // ── TX: 13바이트 TCP 레코드 → 14바이트 CAN 프레임 → sendCanFrame(code 7) ──────────
    /**
     * 14바이트 CAN 프레임 빌드. ID 인코딩 = libcpcomm.so makeCanSendBuffer 역분석 **확정**.
     *   ext(29bit): word = ((canId shl 3) or 0x04)     std(11bit): word = (canId shl 21)
     *   byte[1..4] = word 빅엔디안(MSB first). rtr=0(데이터프레임)만 사용.
     * 검산: 0x06000001 ext → 30 00 00 0C / 0x07000001 ext → 38 00 00 0C.
     */
    private fun makeFrame14(canId: Int, dlc: Int, data: ByteArray): ByteArray {
        val ext = canId > 0x7FF                       // 29비트 확장 ID (Keya 0x06000001/0x07000001 등)
        val id = canId.toLong() and 0xFFFFFFFFL
        val word = (if (ext) ((id shl 3) or 0x04L) else (id shl 21)) and 0xFFFFFFFFL
        val f = ByteArray(14)
        f[0] = channel.toByte()                       // ★ TODO(HW): 채널 실차확인
        f[1] = (word ushr 24).toByte()                // ID워드 big-endian (MSB first)
        f[2] = (word ushr 16).toByte()
        f[3] = (word ushr 8).toByte()
        f[4] = (word and 0xFF).toByte()
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
        // ★ 개통 선행: CAN 안 열렸으면 setCANBaudrate(code3) 먼저(미개통 TX = DeadObject 원인이었음).
        if (!canOpened) {
            Log.i("CpdeviceCan-TX", "CpDev-TX: canOpened=false → setCANBaudrate 먼저")
            openCan()                              // RX 콜백(code1) 자동 등록 안 함(크래시 회피)
        }
        Log.i("CpdeviceCan-TX", "CpDev-TX: ch=%d id=0x%08X dlc=%d data=%s".format(
            channel, canId, dlc, data.copyOf(dlc).joinToString("") { "%02X".format(it) }))
        // 1차 시도 → DeadObject 면 재연결(+콜백 재등록)만 하고 1회 재시도. ★ 개통(code3)은 재실행 안 함:
        //   code3 가 서비스 재시작을 유발하므로 매 DeadObject 마다 부르면 재시작 폭주. 재시작은 CAN 열린 채 복구됨.
        if (doTransactCan(canId, dlc, data, attempt = 1)) return true
        Log.w("CpdeviceCan-TX", "CpDev-TX: retry — 재연결(콜백 재등록)")
        ensureBinder()
        return doTransactCan(canId, dlc, data, attempt = 2)
    }

    /** 단일 CAN 프레임 transact(code 7). alive 체크+재획득 포함. 성공 true. */
    private fun doTransactCan(canId: Int, dlc: Int, data: ByteArray, attempt: Int): Boolean {
        val b = ensureBinder()
        if (b == null) { Log.w("CpdeviceCan-TX", "CpDev-TX: binder=null (no send, attempt=$attempt)"); return false }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        return try {
            val frame = makeFrame14(canId, dlc, data)
            Log.i("CpdeviceCan-TX", "CpDev-TX: 14B=%s".format(frame.joinToString("") { "%02X".format(it) }))
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeInt(1)
            p.writeByteArray(frame)
            val ret = b.transact(CODE_SEND_CAN_FRAME, p, r, 0)
            r.readException()
            lastTxOk = true; txCount++
            Log.i("CpdeviceCan-TX", "CpDev-TX: transact code7 ret=%b (attempt=%d)".format(ret, attempt))
            true
        } catch (e: android.os.DeadObjectException) {
            lastTxOk = false; lastError = "DeadObject"
            binder = null   // 핸들만 폐기(canOpened 유지) → 재연결+콜백재등록으로 복구(개통 code3 재실행 안 함)
            Log.w("CpdeviceCan-TX", "CpDev-TX: DeadObjectException (attempt=$attempt) -> drop handle, 재연결")
            false
        } catch (e: Throwable) {
            lastTxOk = false; lastError = "ERR txTest: ${e.message}"
            val alive = try { binder?.isBinderAlive ?: false } catch (_: Throwable) { false }
            Log.e("CpdeviceCan-TX", "CpDev-TX ERR: transact ${e.message} (alive=$alive, attempt=$attempt)", e)
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
        // ★ 개통 선행: 스트리밍도 CAN 안 열렸으면 setCANBaudrate 먼저.
        if (!canOpened) openCan()   // RX 콜백(code1) 자동 등록 안 함(크래시 회피)
        // 스트림 경로도 단일 핸들/alive 체크 공용 경로 사용(DeadObject 시 자가 복구).
        doTransactCan(canId, dlc, data, attempt = 1)
    }

    // ── RX: registerCallback(code 1) — 우리 Binder 등록, 수신 14B 프레임을 rxQueue 로 ──────
    private val rxCallback = object : Binder() {
        override fun onTransact(code: Int, data: Parcel, reply: Parcel?, flags: Int): Boolean {
            // ★ 진입 즉시 무조건 1줄(code 무관). RX 콜백 transact code 는 콜백쪽 구현이라 고정값 가정 금지 →
            //   어떤 code 든 [int 헤더 + byte[14×N]] 프레임형 페이로드면 파싱(아니면 raw 덤프). 예외에도 안 죽게.
            try {
                val dataSize = try { data.dataSize() } catch (_: Throwable) { -1 }
                Log.i("CpdeviceCan-RX", "CpDev-RX: code=%d size=%d flags=%d".format(code, dataSize, flags))
                rxCount++
                var parsed = false
                try {
                    val hdr = data.readInt()                   // count/timestamp 추정 — 무시
                    val buf = data.createByteArray()
                    if (buf != null && buf.size >= 14) {
                        val nFrames = buf.size / 14
                        Log.i("CpdeviceCan-RX", "code=%d hdr=%d bytes=%d -> %d frames".format(code, hdr, buf.size, nFrames))
                        for (i in 0 until nFrames) {
                            val o = i * 14
                            val ch = buf[o].toInt() and 0xFF
                            val dlc = (buf[o + 5].toInt() and 0xFF).coerceIn(0, 8)
                            // ID워드 빅엔디안 → canId 복원(makeFrame14/makeCanSendBuffer 역). ext = word&0x04.
                            val word = ((buf[o + 1].toLong() and 0xFF) shl 24) or
                                       ((buf[o + 2].toLong() and 0xFF) shl 16) or
                                       ((buf[o + 3].toLong() and 0xFF) shl 8) or
                                        (buf[o + 4].toLong() and 0xFF)
                            val canId = (if (word and 0x04L != 0L) (word ushr 3) and 0x1FFFFFFFL
                                         else (word ushr 21) and 0x7FFL).toInt()
                            val payload = buf.copyOfRange(o + 6, o + 6 + dlc)
                            Log.i("CpdeviceCan-RX", "RX ch=%d id=0x%08X dlc=%d data=%s".format(
                                ch, canId, dlc, payload.joinToString("") { "%02X".format(it) }))
                            rxQueue.offer(canId to payload)     // push to existing TCP bridge RX record
                        }
                        parsed = true
                    }
                } catch (e: Throwable) {
                    Log.w("CpdeviceCan-RX", "frame parse failed: ${e.message}")
                }
                if (!parsed) dumpRaw(data)                     // 포맷 다르면 분석용 raw 1줄
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
        val b = binder    // ★ ensureBinder 호출 금지(재연결→재등록 재귀 방지). 호출자가 binder 보장.
        Log.i(TAG, "CpDev: registerCallback(code=$CODE_REGISTER_CALLBACK, IBinder) desc=\"$DESCRIPTOR\" binder=${b != null} canOpened=$canOpened")
        if (b == null) { Log.e(TAG, "CpDev ERR: registerCallback binder=null (getService failed)"); return }
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeStrongBinder(rxCallback)
            val ret = b.transact(CODE_REGISTER_CALLBACK, p, r, 0)
            r.readException()
            Log.i(TAG, "CpDev: registerCallback(code1) ret=$ret (true=delivered) — RX 콜백 역transact 대기")
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
