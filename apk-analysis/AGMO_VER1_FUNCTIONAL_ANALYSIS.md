# AGMO Solution ver1 — 기능화 분석 리포트 (세션 핸드오프 · 자급식)

> **이 문서를 읽는 세션에게**: 디컴파일 원본(`app-decompiled` private 레포)은 당신 환경에 **없다**.
> 이 문서가 디컴파일을 직접 리뷰해 추출한 **모든 동작 사실**을 담은 단일 출처다. 디컴파일 파일을
> 다시 찾을 필요 없이 이 문서만으로 farmmachine 구현에 착수하라.
>
> **clean-room 준수**: AGMO 소스·리소스 표현 복제 없음. GPIO 번호·CAN ID·JNI 시그니처·sysfs 경로
> 같은 **인터페이스/하드웨어 사실**만 인용(저작권 비보호 영역, legal/INDEPENDENT_DEVELOPMENT_RECORD.md
> 2-2절). 본 문서의 모든 코드 스니펫은 그 사실에 기반한 **farmmachine 자체 구현 제안**이다.
>
> **분석 방법**: JADX 디컴파일된 Java(2273개 파일) 전수 grep + 핵심 클래스 정독.
> 분석한 핵심 파일: `com/van/jni/VanMcu.java`, `com/van/jni/VanCmd.java`, `com/van/dev/Gpio.java`,
> `com/cp/cputils/{Apollo2,ApolloPro,Spring}.java`,
> `com/agmo/autokit/{CustomActivity,DeviceManager,RDDF,PathManager,PathItem,BoundaryPoint,DegreeTMConverter,MapManager,BuildConfiguration}.java`.

---

## 0. 앱·하드웨어 구조 (필수 전제)

AGMO 앱 `com.agmo.autokit`은 **Qt 기반 앱**이다 (`CustomActivity extends
org.qtproject.qt.android.bindings.QtActivity`). 따라서:

- **조향 제어 루프 · GNSS 수신/파싱 · EKF는 전부 Qt C++ native `.so`** 안에 있다. 이 `.so`는 ARM
  네이티브라 JADX Java 디컴파일에 **나오지 않는다**(libs는 `.gitignore` 제외). → **Java 레이어에는
  NMEA/Unicore 파서·시리얼 오픈·조향 PID가 전혀 없다.** (전수 grep으로 확인: `/dev/tty*`·`UsbManager`·
  `UsbSerial`·NMEA·Socket 0건)
- Java 레이어가 실제 담은 것: 경로 파일 관리(S3 RDDF), Naver 지도 표시, 대시캠 녹화(Quectel QCar),
  디바이스 전원/CAN **브릿지(JNI 선언)**, GNSS/CAN **전원·리셋 GPIO 제어**.

### 하드웨어 플랫폼 — 디컴파일에서 확인된 2개 변종

`com.cp.cputils`에 디바이스 추상화 클래스가 2개 있고, **GNSS 수신기 종류가 다르다**:

| | **Apollo2** (★ ver1 자율조향 디바이스) | ApolloPro (구형/대시캠) |
|---|---|---|
| SoC | Rockchip **RK3568** (`ADC_RK3568_*`, `fe8a0000.usb2-phy`) | MTK (`7000000.ssusb`, `/dev/gpio_dev` ioctl) |
| GPIO 제어 | 표준 Linux sysfs `/sys/class/gpio/gpioNN/value` | `/dev/gpio_dev` + 매직코드(100007 등) |
| **GNSS 수신기** | **Unicore UM482** (`UM482_PWREN=gpio137`) ← 듀얼안테나 헤딩 보드 | u-blox (`setUblox()` 코드 100008/100009) |
| CAN 채널 | **CAN0/CAN1/CAN2 3채널** | 단일 |

**farmmachine 타깃("ver1 듀얼안테나, base=좌/rover=우, 중앙 IMU")은 Apollo2 변종 = Unicore UM482**와
정확히 일치. → **이전 리포트의 "GNSS=u-blox" 표기는 정정한다. ver1 자율조향 디바이스의 GNSS는
Unicore UM482(듀얼안테나 RTK 헤딩 보드)다.** (u-blox는 구형 대시캠 ApolloPro 변종)

#### Apollo2 GPIO 맵 (RK3568, 하드웨어 사실 — `Apollo2.java`)

```
GNSS 관련:
  UM482_PWREN   = gpio137   # UM482 보드 전원 (★ 듀얼안테나 GNSS)
  GNSS_LNA_EN   = gpio101   # GNSS 안테나 LNA 전원
  GNSS_RST_N    = gpio136   # GNSS 리셋 (active-low)
  STATE4_LNA_PWR= gpio23    # LNA 전원 상태 읽기

CAN 관련:
  CAN_PWR_EN    = gpio61    # CAN 트랜시버 전원 (공통)
  CAN0_ON_EN0   = gpio99    # CAN0 enable
  CAN1_ON_EN1   = gpio154   # CAN1 enable
  CAN2_ON_EN2   = gpio128   # CAN2 enable

기타:
  RS_485_EN     = gpio134   # RS485 트랜시버
  ADC_RK3568_A  = iio:device0/in_voltage4_raw  # 아날로그 입력 4 (★ WAS 각도센서 가능성)
  ADC_RK3568_B  = iio:device0/in_voltage5_raw  # 아날로그 입력 5
  USBHOST       = fe8a0000.usb2-phy/otg_mode   # "host"/"peripheral" 스위치
  FUN_KEY1/2    = gpio20/21  # 물리 버튼 (데드맨/기능키 후보)
  OUTPUT1/2     = gpio135/15
```

→ farmmachine 함의: CAN 쓰기 전에 **`CAN_PWR_EN`(gpio61) + 해당 채널 enable GPIO**를 켜야 트랜시버가
산다. `ADC voltage4/5`는 **WAS(휠 각도센서) 아날로그 피드백** 경로일 수 있다(Keya 홀센서 CAN
피드백과 별개일 가능성 — 시운전에서 확인). 듀얼안테나는 base/rover가 아니라 **UM482 단일 보드의
ANT1(primary)/ANT2(secondary)** 구성이다.

---

## [1] GNSS 데이터 경로 — ⭐ 미동작 1차 원인

### AGMO 동작 사실
- USB-serial/Android USB API/Location API/NMEA 문자열 모두 **Java에 0건**. GNSS 읽기는 Qt native.
- 확정 사실: **GNSS 수신기 = Unicore UM482**(`UM482_PWREN` GPIO + 전용 전원 시퀀스). UM482는
  **2-안테나 RTK 헤딩 보드**로, 위치(BESTPOS류) + **헤딩을 보드가 직접 계산**해 출력한다.
- 데이터 경로(추론, native 영역): **UM482 보드 → RK3568 내부 UART(tty) → Qt native에서 파싱.**
  UM482 출력은 **NMEA + Unicore 독자 메시지**(아래 [2]).
- 전원 시퀀스 힌트: `setCanPwrEn`처럼 GNSS도 `UM482_PWREN`→`GNSS_LNA_EN`→`GNSS_RST_N` 순으로
  켜고 리셋 해제. RTCM 보정은 보드로 주입(LoRa/NTRIP 경유).

### farmmachine 수정 지점
- **현 상태**: `auto-steering/src/`에 `autosteer_core.py`만 존재. `f9p_client.py`·`app_main.py`·
  `vendor_profiles.py`는 **없다.** `StateEstimator.update_rtk(lat, lon, quality)`는 정의됐으나
  **호출하는 코드가 0곳** → GNSS를 읽어 EKF에 넣는 경로가 통째로 없음 = 미동작 1차 원인.

작업:
1. **`auto-steering/src/gnss_client.py` 신규** (이름은 `f9p_client.py`로 둬도 무방하나 칩이 UM482이므로
   다중 수신기 지원 권장). 파서 2종을 모두 지원:
   - **Unicore UM482**(ver1 실기기): NMEA(`GGA`,`RMC`,`GNHDT`) + Unicore 독자(`#BESTPOSA`,
     `#HEADINGA`/`#UNIHEADINGA`, `#GPHPDA`/`GPHPR`). 헤딩은 `#HEADINGA`/`GPHPR`에서 직접 취득.
   - **u-blox F9P**(farmmachine CLAUDE.md 보유장비, 대안): NMEA(`GGA`,`RMC`) + UBX
     (`UBX-NAV-PVT`, `UBX-NAV-RELPOSNED`).
   - 공통 콜백: `on_rtk(lat, lon, quality, height)`, `on_heading(heading_deg, pitch_deg, baseline_len_m, sol_status)`.
   - 품질 매핑: GGA fix quality `4=RTK Fixed`, `5=RTK Float` → `SafetyMonitor`(4/5만 허용)와 그대로 정합.
2. **`android/app/.../usb/GnssUsbSerial.kt` 신규** — `UsbManager.openDevice()` + CDC-ACM/FTDI/CP210x
   바이트 스트림 → Python 파서로 전달. (UM482가 USB-CDC면 그대로, UART면 CAN과 같은 native 브릿지 필요.
   farmmachine는 USB 직결 가정이므로 USB-serial 경로로 구현.)
3. **`auto-steering/src/app_main.py` 신규** — `start_gnss()`에서 클라이언트 생성,
   `on_rtk → estimator.update_rtk`, `on_heading → estimator.update_heading`([2]) 배선 + engage 게이트.

---

## [2] 듀얼안테나 base/rover · 헤딩 오프셋

### AGMO 동작 사실
- 헤딩 오프셋 상수(`mountAngle`/`installAngle`)는 native 영역(Java 0건). 단 하드웨어가 **UM482 듀얼안테나**임이
  확정 → 헤딩 산출 모델이 명확:
  - UM482는 **ANT1=primary(위치), ANT2=secondary(헤딩 슬레이브)**. 보드가 ANT1→ANT2 베이스라인
    방위를 `#HEADINGA`(Unicore) 또는 `GPHPR`/`GNHDT`로 출력. (F9P moving-base면 동등하게
    `UBX-NAV-RELPOSNED.relPosHeading`, RTCM 받는 쪽=rover가 헤딩 주체.)

### farmmachine 수정 지점
- **현 상태**: `autosteer_core.py` EKF 상태 `[x, y, heading, speed, angular_vel]`에서 heading은
  **`update_imu(raw_heading, …)`의 IMU yaw로만** 보정. `on_heading_meas`/`dual_baseline_offset_deg`/
  `dual_roll_sign`은 **코드에 없음** → 신규 추가.

작업:
1. **`StateEstimator.update_heading(gnss_heading_rad)` 신규** — GNSS 절대 헤딩을 EKF 측정 업데이트로
   추가(IMU yaw보다 절대 정확, 드리프트 없음). residual `_wrap(gnss_heading - x[2])`, R≈(0.5°)².
   RTK Fixed일 때만 신뢰(품질 게이트).
2. **헤딩 오프셋** — 가로 베이스라인, 진행방향 기준 base/ANT1=좌, rover/ANT2=우면 보드가 출력하는
   베이스라인 방위는 차량 우측(진행방향 +90°)을 가리킴. 따라서:
   ```
   vehicle_heading = gnss_baseline_heading - dual_baseline_offset_deg   # 좌→우 가로배치 → 90
   ```
   farmmachine 계획값 `dual_baseline_offset_deg=90`은 이 배치와 정합. **부호(+90 vs -90)는 첫 시운전 때
   알려진 진행방향(예: 직진 주행 코스)으로 반드시 검증.**
3. **`dual_roll_sign`** — 좌우 안테나는 차량 roll에 따라 높이차 발생 → 헤딩에 미세 오차/롤 추정 가능.
   UM482 `#HEADINGA`의 pitch 필드(또는 F9P `relPosD`)로 roll 보정. 평지 정지 시 pitch≈0 확인으로 캘리브레이션.
   `dual_roll_sign=+1` 기본, 안테나 좌우 반대로 달면 -1.
4. **`auto-steering/src/vendor_profiles.py` 신규** — `AGMO_VER1` 프로파일:
   ```python
   AGMO_VER1 = VendorProfile(
       gnss_receiver="unicore_um482",      # ver1 실기기
       dual_antenna=True,
       antenna_layout="lateral",            # 가로 베이스라인
       base_antenna="left", rover_antenna="right",
       dual_baseline_offset_deg=90,         # 시운전 검증 필수
       dual_roll_sign=+1,
       heading_msg=["#HEADINGA", "GPHPR", "GNHDT"],
   )
   ```

---

## [3] 모터/CAN — 교차검증 중 ⭐ 치명적 버그 (미동작 2차 원인)

### AGMO 동작 사실 — `libsysmcu.so` 인터페이스 (VanMcu.java 전수 확인)

CAN은 `com.van.jni.VanMcu`(→ `System.loadLibrary("sysmcu")`)가 소유. **채널 기반 API, socketcan/`can0`
아님.** 디컴파일에서 확인한 **전체 native 시그니처**:

```
// TX / 설정 (native)
boolean CanWrite(int channel, int id, byte[] data)
boolean setCanSpeed(int channel, int speed)
boolean CanFilterCtrl(int, int)
boolean CanHwFilterAdd(int ch, int id, int mask)   boolean CanHwFilterClear(int ch)
boolean CanSwFilterAdd(int ch, int id, int mask)    boolean CanSwFilterClear(int ch)
int     getCanCount()    int getCanSpeed(int ch)
// 전원/IO (native): PowerCtrl, OutputSet, InputGet, getPowerVoltage, getTemperature, getAccState, getVersion ...

// 확장프레임 상수 (static)
CAN_EFF_FLAG = 0x80000000      // 29-bit 확장 ID
CAN_RTR_FLAG = 0x40000000

// 콜백 활성 필터 비트마스크 (private native setCallback(int))
ACC=1, CAN=2, INPUT=4, DEBUG=8
```

**RX 콜백 메커니즘 (← farmmachine가 틀린 부분, 정확히 기록):**
1. `setOnCanListener(listener)`는 **순수 Java 메서드(native 아님)**. 내부에서 비트마스크를 갱신해
   `setCallback(int bitmask)`(이게 native)를 호출한다. CAN 수신하려면 비트 `2`(CAN)가 켜져야 한다.
2. native(.so)는 이벤트 발생 시 **`VanMcu.onCallback(int type, byte[] data)` 라는 static 메서드를
   역호출(JNI CallStaticVoidMethod)** 한다. 이 메서드가 type별로 디멀티플렉싱한다.
3. **CAN 프레임(type==2) 바이트 패킹** (onCallback이 디코드하는 실제 레이아웃):
   ```
   data[0]      = channel
   data[1..4]   = CAN id (BIG-ENDIAN, 4바이트)
   data[5]      = DLC (payload 길이)
   data[6..6+DLC] = payload
   ```
   (ACC: data[0]=상태 / INPUT: data[0],data[1] / DEBUG: new String(data))

### farmmachine 수정 지점 — 현재 CAN 수신이 **무조건 깨짐**

`android/app/src/main/java/com/farmmachine/autosteer/can/VanMcu.kt`가 실제 `.so`와 불일치.
실기기는 진짜 `libsysmcu.so`를 로드하므로 → `UnsatisfiedLinkError` 또는 콜백 영구 미발생:

| farmmachine VanMcu.kt (현재) | 실제 libsysmcu.so | 결과 |
|---|---|---|
| `external fun setCallback(enable: Boolean)` | native `setCallback(int 비트마스크)` | **타입 불일치 → 링크 실패** |
| `external fun setOnCanListener(listener)` | **순수 Java**(native 심볼 없음) | **`UnsatisfiedLinkError`** |
| `interface OnCanListener.onReceive(...)`로 수신 가정 | native는 `onCallback(int,byte[])` **static** 역호출 | **수신 콜백 영구 미발생** |
| `onCallback(int,byte[])` static **없음** | native가 이 심볼을 찾아 호출 | **디멀티플렉싱 불가 → heartbeat 0건** |

→ `ApolloCanBus.kt`의 `onHeartbeat`(Keya KY170 `0x07000001` 피드백)가 **실기기에서 절대 안 들어옴.**
조향각 피드백이 없으니 `SteeringActuator` 위치 루프가 닫히지 않음 = 모터 closed-loop 제어 미동작.
**[1] GNSS 부재와 함께 미동작의 2차 핵심 원인.**

수정안(`VanMcu.kt`, 자체 구현 — 시그니처를 .so에 정확히 맞춤):
```kotlin
object VanMcu {
    init { System.loadLibrary("sysmcu") }

    @JvmStatic external fun CanWrite(channel: Int, id: Int, data: ByteArray): Boolean
    @JvmStatic external fun setCanSpeed(channel: Int, speed: Int): Boolean
    @JvmStatic private external fun setCallback(filterBitmask: Int): Boolean   // ★ Boolean→Int

    const val FILTER_CAN = 2
    private var canListener: ((ch: Int, id: Int, data: ByteArray) -> Unit)? = null

    fun setOnCanListener(cb: (Int, Int, ByteArray) -> Unit) {   // ★ 순수 Kotlin, external 제거
        canListener = cb
        setCallback(FILTER_CAN)                                 // CAN 비트 활성
    }

    @JvmStatic fun onCallback(type: Int, data: ByteArray) {     // ★ native가 역호출 — 이름/시그니처 고정
        if (type == FILTER_CAN) {
            val ch  = data[0].toInt()
            val id  = ((data[1].toInt() and 0xFF) shl 24) or ((data[2].toInt() and 0xFF) shl 16) or
                      ((data[3].toInt() and 0xFF) shl 8)  or  (data[4].toInt() and 0xFF)   // big-endian
            val dlc = data[5].toInt()
            canListener?.invoke(ch, id, data.copyOfRange(6, 6 + dlc))
        }
    }
}
```
부수 작업:
- `ApolloCanBus.start()`는 새 람다 시그니처 `setOnCanListener { ch, id, data -> … }`에 맞춰 수정.
- **CAN 채널 번호 확인 필수**: Apollo2는 CAN0/1/2 3채널. farmmachine `channel=0` 기본값이 모터가 물린
  실제 채널과 다를 수 있음 → 시운전에서 `getCanCount()`/스캔으로 확정하고 설정값으로 노출.
- CAN 전원: 쓰기 전에 `CAN_PWR_EN(gpio61)` + 해당 채널 `CANx_ON_ENx` GPIO ON 필요(아래 [4] 부팅 시퀀스).
- 29-bit 확장 ID(`0x06000001`): 첫 통신에서 `id or CAN_EFF_FLAG(0x80000000)` 필요 여부 확인.
- 정합 확인된 값(수정 불필요): bitrate `250_000` ✓, TX/RX/HB `0x06000001`/`0x05800001`/`0x07000001` ✓,
  heartbeat 파싱(angle big-endian, fault 비트맵) ✓.

---

## [4] engage / 주행 제어 흐름 + 데이터 모델

### AGMO 동작 사실
- 제어 루프 진입부는 Qt native(코드 비추출). 데이터 모델·부팅 시퀀스 사실만:
- **부팅/초기화**(`CustomActivity.onCreate`): 부트컴플리트 자동 실행(`AutoRun`), `com.van.service`로
  `CAMERGPIOON` 브로드캐스트(하드웨어 GPIO를 별도 시스템 서비스가 켬), Device Admin 등록 +
  **uninstall 차단 + 상태바 비활성**(키오스크 모드). → farmmachine도 CAN/GNSS 하드웨어 인에이블을
  명시적으로 켜는 초기화 단계 필요.
- **경로 = RDDF**(`RDDF.java`): `route: List<LatLng>` + `speeds: List<Double>` +
  `implement_commands: List<Boolean>` 3배열 병렬 → farmmachine `Waypoint(x,y,speed,implement_down,section)`와 1:1.
- **RDDF 파일 포맷**(`PathManager.parseRddfFromFile`): 탭(`\t`) 구분, 행당:
  `col[1],col[2]=TM 좌표`(평문 double 아니면 **AES256 복호화**), `col[6]=speed`, `col[7]=implement("1"=내림)`.
  파일명 접두 `AL/AC/ME/FP` = ABLine/ABCurve/Memory/FullPath. 10000점 초과 시 1/10 다운샘플(`simplifyRoute`).
- **PathItem JSON**(S3 저장): `{meta_data:{name,code,user,date,rddf,map}, info:{mode,total_time,
  total_distance,bookmark,boundary_points}}`. `boundary_points`=필드 경계 폴리곤, 각 점이
  `[AES256(x), AES256(y), AES256(z)]` — **z=고도** 포함(3D, 듀얼안테나 높이와 연계).
- **좌표계**(`DegreeTMConverter`): 한국 TM(GRS80, e²=0.00669437999013, 원점경도 **127°E**,
  false easting **200000**, false northing **600000**). 위경도↔평면(m) 변환 → 추종은 **평면 m에서 수행**, 표시만 위경도.
- 주행 모드 4종 → farmmachine 매핑: ABLine→`ABLineStrategy`, ABCurve→`ContourStrategy`,
  Memory(주행기록 재생)→`CustomStrategy`, FullPath→전체경로.

### farmmachine 수정 지점
1. **좌표 변환 모듈** — `gnss_client` 뒤단에 위경도→로컬 평면 투영 추가. AGMO 한국 TM 상수를 그대로 쓸
   필요 없음 — **작업 농지 중심 기준 로컬 ENU 투영**으로 자체 구현 권장(차용 사실: "위경도가 아닌 평면
   m에서 추종"). EKF/경로추종은 평면에서.
2. **하드웨어 초기화 시퀀스**(Kotlin, `MainActivity` 또는 신규 `HardwareInit.kt`): engage 전에
   `CAN_PWR_EN`+채널 enable, `UM482_PWREN`+`GNSS_LNA_EN`+`GNSS_RST_N`(리셋 해제), `RS_485_EN`(LoRa NTRIP용)을
   켠다. (Apollo2는 sysfs echo, root/시스템 권한 필요 — Device Admin/시스템앱 서명.)
3. **engage 안전 게이트**(`app_main.py`): `SafetyMonitor`(데드맨 + RTK 품질 4/5 + 속도≤2.5m/s)
   **AND** GNSS 헤딩 유효(RTK Fixed) **AND** CAN heartbeat 수신 중([3] 수정 후라야 성립).
   heartbeat 끊김/전류 초과 시 즉시 disengage. 물리버튼 `FUN_KEY1/2`(gpio20/21)를 데드맨/engage 토글로 활용 가능.
4. **제어 주기** — RTK 10Hz/IMU 100Hz 융합. 조향 명령은 100Hz 또는 모터 watchdog과 조화.
   `ApolloCanBus.resetWatchdog` 800ms keepalive 유지하되 정상 제어 주기는 그보다 짧게.

---

## 결론 — 미동작 3대 근본 원인 (착수 우선순위)

| # | 근본 원인 | 고칠 파일 | 난이도 |
|---|---|---|---|
| 1 | **CAN RX 콜백이 .so와 불일치 → heartbeat 영구 미수신** (조향 피드백 루프 안 닫힘) | `android/app/.../can/VanMcu.kt` (setCallback Int화 + `onCallback` static 추가 + big-endian id 파싱) | **小·확실** |
| 2 | **GNSS 입력 코드 자체가 없음** — `gnss_client.py`/`app_main.py` 부재, `update_rtk` 호출 0곳 | `gnss_client.py`·`app_main.py`·`GnssUsbSerial.kt` (모두 신규) | 中 |
| 3 | **듀얼안테나 헤딩 입력 경로 없음** — EKF가 IMU yaw로만 헤딩 추정 | `autosteer_core.py`(`update_heading`+오프셋), `vendor_profiles.py`(신규) | 中 |

**확정 하드웨어/프로토콜 사실 요약**
- ver1 자율조향 디바이스 = **Apollo2(RK3568)**, GNSS = **Unicore UM482 듀얼안테나 헤딩 보드**
  (NMEA + Unicore `#HEADINGA`/`GPHPR`). ※ u-blox는 구형 대시캠 ApolloPro 변종.
- CAN = **`libsysmcu.so` 채널 API**(250k, **CAN0/1/2 3채널**, big-endian id 패킹,
  **`onCallback(int,byte[])` static 역호출**, 필터 비트마스크 CAN=2, EFF=0x80000000), socketcan 아님.
- 하드웨어 인에이블: `CAN_PWR_EN=gpio61`, `CANx=gpio99/154/128`, `UM482_PWREN=gpio137`,
  `GNSS_LNA_EN=gpio101`, `GNSS_RST_N=gpio136`, `RS_485_EN=gpio134`. `ADC voltage4/5`=아날로그(WAS 후보).
- 경로 = **RDDF**(탭구분, TM 평면, col1/2=좌표·col6=속도·col7=작업기, AES256 옵션) + **PathItem JSON**
  (boundary_points = 3D 경계폴리곤). 좌표계 = 한국 TM(GRS80, 원점 127°E).
- 조향 루프·GNSS 파서 = Qt C++ native (소스 비추출, clean-room 유지).

**권장 착수 순서**: [1번 표] VanMcu.kt 수정(가장 확실·작음) → GNSS 클라이언트(UM482) + app_main 배선 →
EKF 헤딩 입력. 셋이 모두 들어가야 실기기에서 "위치 + 헤딩 + 조향피드백" 폐루프가 처음으로 닫힌다.
