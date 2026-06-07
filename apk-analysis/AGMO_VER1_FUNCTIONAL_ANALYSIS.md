# AGMO Solution ver1 — 기능화 분석 리포트 (세션 핸드오프)

> **작성 목적**: AGMO Solution ver1 앱(디컴파일물, private)을 분석해, UI만 비슷하고
> 실기기에서 기능이 안 도는 farmmachine 자율조향 앱을 "실제로 동작"하게 만들기 위한
> **동작 사실(interface/protocol)**을 추출한 핸드오프 문서. 다른 세션이 이 문서만 읽고
> 바로 구현에 착수할 수 있도록 정리.
>
> **clean-room 준수**: AGMO/CHCNAV/FJD 소스·리소스 표현을 복제하지 않음.
> 본 문서의 모든 코드 스니펫은 인터페이스 사실에 기반한 **farmmachine 자체 구현 제안**.
> (CLAUDE.md 법적 준수 사항 + legal/INDEPENDENT_DEVELOPMENT_RECORD.md 2-2절 근거)

---

## 0. 전제 — 분석으로 밝혀진 앱 구조

AGMO 앱 패키지 `com.agmo.autokit`은 **Qt 기반 앱**이다
(`CustomActivity extends org.qtproject.qt.android.bindings.QtActivity`). 그래서:

- **조향 제어 루프 · GNSS 수신/파싱 · EKF는 전부 Qt C++ native `.so`** 안에 있다.
  이 `.so`는 ARM 네이티브이고 디컴파일 결과(`app-decompiled/sources/`)에 **포함돼 있지 않다**
  (`.gitignore`가 libs/resources 제외). → **Java 레이어에는 NMEA 파서·시리얼 오픈·조향 PID가 전혀 없다.**
- Java 레이어가 실제로 담은 것: 경로 파일 관리(S3 RDDF), Naver 지도 표시, 대시캠 녹화(Quectel QCar),
  디바이스 전원/CAN **브릿지 선언**, GNSS **전원/리셋 GPIO 제어**.
- 하드웨어는 **Allwinner(sunxi) SoC** 기반. CAN·GPIO는 `libsysmcu.so` + 별도 시스템 서비스
  `com.van.service`가 소유.

**결론**: 정확한 tty 경로·baud·파싱 코드는 Java로 단정 불가(native 영역). 그러나 **GNSS 칩 정체·
전기적 경로·CAN 콜백 메커니즘**이라는 더 결정적인 사실을 확보했고, 이것이 farmmachine 미동작의
실제 원인과 직결된다.

분석 근거 파일(app-decompiled, 인터페이스 사실만 인용):
`com/van/jni/VanMcu.java`, `com/cp/cputils/Spring.java`, `com/cp/cputils/Apollo2.java`,
`com/agmo/autokit/{CustomActivity,RDDF,PathManager,DegreeTMConverter,MapManager}.java`.

---

## [1] GNSS 데이터 경로 — ⭐ 미동작 1차 원인

### AGMO 동작 사실
- `/dev/tty*`, `UsbManager`, `UsbSerial`, CDC-ACM, Android Location API, NMEA 문자열 —
  **Java 레이어에 0건**(전수 grep). GNSS 데이터 읽기는 Qt native에 있음.
- 단, GNSS 하드웨어 제어가 sysfs 노드로 노출됨(`Spring.java`, sunxi 플랫폼):

  | sysfs 경로 | 의미 |
  |---|---|
  | `/sys/devices/virtual/misc/sunxi-gps/rf-ctrl/nstandby_state` | **`UBLOX`** standby 해제 (`setUblox()`) |
  | `/sys/class/misc/sunxi-gps/rf-ctrl/gnss_pwren_state` | GNSS 전원 enable |
  | `/sys/devices/virtual/misc/sunxi-gps/rf-ctrl/max485_state` | RS485(MAX485) 트랜시버 |
  | `/sys/class/misc/sunxi-gps/rf-ctrl/usbid_con_state` | USB host/OTG 스위치 |

  `Apollo2.java`엔 `setGnssLnaEn()`, `setGnssRstN()`(LNA/리셋 제어)도 존재.

- **결정적**: 변수명 `UBLOX` + 전용 `setUblox()` → **AGMO ver1 GNSS 칩은 u-blox 계열**
  (= farmmachine ZED-F9P와 동일 패밀리). GPIO 전원/standby/리셋 형태는 **내부 UART 직결 u-blox
  모듈**의 전형. 데이터 경로:

  > **u-blox 모듈 → SoC 내부 UART(tty) → Qt native에서 NMEA + UBX 파싱.** (USB Android API/CAN 아님)

### farmmachine 수정 지점
- **현 상태**: `f9p_client.py`·`app_main.py`·`vendor_profiles.py`가 **존재하지 않음.**
  `autosteer_core.py`의 `StateEstimator.update_rtk(lat, lon, quality)`는 정의돼 있으나
  **호출하는 코드가 0곳** → GNSS를 읽어 EKF에 넣는 경로가 통째로 없음. = 미동작 1차 원인.
- farmmachine은 GNSS가 **USB 직결 F9P**(CLAUDE.md)이므로 AGMO의 내부 tty가 아니라
  **Android USB-serial(CDC-ACM)** 경로로 구현해야 함.

작업:
1. **`auto-steering/src/f9p_client.py` 신규** — u-blox 파서(칩 정체 확정됨).
   - NMEA: `GGA`(위치+fix quality), `RMC`(속도+코스/COG).
   - UBX: `UBX-NAV-PVT`(위치+속도), `UBX-NAV-RELPOSNED`(듀얼안테나 heading, → [2]).
   - 콜백 2개: `on_rtk(lat, lon, quality)`, `on_heading(heading_deg, baseline_len_m, flags)`.
   - 품질 매핑: GGA fix quality `4=RTK Fixed`, `5=RTK Float` → `SafetyMonitor`(4/5만 허용)와 그대로 정합.
2. **`android/app/.../usb/F9pUsbSerial.kt` 신규** — `UsbManager.openDevice()` + CDC-ACM(또는 FTDI/CP210x)
   바이트 스트림 → Python 파서 전달. Apollo USB host 모드 스위칭이 필요할 수 있음(위 `usbid_con_state` 힌트).
3. **`auto-steering/src/app_main.py` 신규** — `start_gnss()`에서 F9pClient 생성,
   `on_rtk → estimator.update_rtk`, `on_heading → estimator.update_heading`([2]) 배선.

---

## [2] 듀얼안테나 base/rover · 헤딩 오프셋

### AGMO 동작 사실
- `relPosHeading`/`mountAngle`/`installAngle` 상수는 **Java에 없음**(native).
- GNSS가 u-blox이므로 듀얼안테나 heading은 **UBX-NAV-RELPOSNED**의 `relPosHeading`
  (base→rover 벡터 방위, 1e-5 deg 단위)에서 나옴. RTCM 받는 쪽 = rover(moving-base에서 heading 주체).

### farmmachine 수정 지점
- **현 상태**: `autosteer_core.py` EKF 상태 `[x, y, heading, speed, angular_vel]`에서 heading은
  **`update_imu(raw_heading, …)`의 IMU yaw로만** 보정됨. `on_heading_meas`/`dual_baseline_offset_deg`/
  `dual_roll_sign`은 **현재 코드에 없음** → 신규 추가.

작업:
1. **`StateEstimator.update_heading(gnss_heading_rad)` 신규** — GNSS 절대 heading EKF 측정 업데이트
   (IMU yaw보다 절대 정확). residual `_wrap(gnss_heading - x[2])`, R≈(0.5°)².
2. **헤딩 오프셋** — 가로 베이스라인, base=좌/rover=우(진행방향 기준)면 `relPosHeading`은 차량 우측을
   가리킴 = 진행방향 + 90°. 따라서:
   ```
   vehicle_heading = relPosHeading - dual_baseline_offset_deg   # 좌/우 가로배치 → 90
   ```
   farmmachine 계획값 `dual_baseline_offset_deg=90`은 정합. **부호(90 vs -90)는 첫 시운전 때
   알려진 진행방향으로 반드시 검증.**
3. **`dual_roll_sign`** — 좌우 안테나는 roll에 따라 높이차 발생 → heading 미세오차.
   RELPOSNED `relPosD`(Down 성분)로 roll 추정·보정. 평지 정지 시 `relPosD≈0` 확인으로 캘리브레이션.
4. **`auto-steering/src/vendor_profiles.py` 신규** — `AGMO_VER1` 프로파일에
   `gnss_chip="u-blox"`, `dual_antenna=True`, `baseline_orientation="lateral_left_base"`,
   `dual_baseline_offset_deg=90`, `dual_roll_sign=+1` 등 상수 집약.

---

## [3] 모터/CAN — 교차검증 중 ⭐ 치명적 버그 발견 (미동작 2차 원인)

### AGMO 동작 사실 (libsysmcu.so 인터페이스)
`com.van.jni.VanMcu`(→ `libsysmcu.so`)가 CAN 소유. **채널 기반 API, socketcan/`can0` 아님.**
- TX: `CanWrite(int channel, int id, byte[] data)`
- 속도: `setCanSpeed(int channel, int speed)`
- 확장 프레임 상수: `CAN_EFF_FLAG = 0x80000000`, `CAN_RTR_FLAG = 0x40000000`
  (Keya `0x06000001`은 29-bit 확장 → 첫 통신에서 EFF OR 필요 여부 확인)
- **RX 콜백 메커니즘(핵심)**:
  1. `setOnCanListener(...)`는 **순수 Java**(native 아님). 내부에서 비트마스크 필터 만들어
     `setCallback(int)`(← 이게 native) 호출.
  2. 필터 비트마스크: `ACC=1, CAN=2, INPUT=4, DEBUG=8`. CAN 수신하려면 **CAN 비트(2)** 켜야 함.
  3. native가 **`VanMcu.onCallback(int type, byte[] data)` static 메서드를 역호출**해 이벤트 전달.
  4. CAN 프레임 패킹: `data[0]=channel`, `data[1..4]=id`(**big-endian**), `data[5]=DLC`, `data[6..]=payload`.

### farmmachine 수정 지점 — 현재 RX가 **무조건 깨짐**
`android/app/.../can/VanMcu.kt`가 실제 `.so` 시그니처와 불일치. 실기기는 진짜 `libsysmcu.so`를
로드하므로 불일치 시 `UnsatisfiedLinkError` 또는 콜백 영구 미발생:

| farmmachine VanMcu.kt (현재) | 실제 libsysmcu.so | 결과 |
|---|---|---|
| `external fun setCallback(enable: Boolean)` | `setCallback(int)` 비트마스크 | 시그니처 불일치 → 링크 실패/오작동 |
| `external fun setOnCanListener(listener)` | **순수 Java**(native 아님) | `UnsatisfiedLinkError` |
| `interface OnCanListener.onReceive(...)` 수신 가정 | native는 `onCallback(int,byte[])` static 역호출 | **수신 콜백 영구 미발생** |
| (없음) | `onCallback(int, byte[])` static 필수 | 누락 → 디멀티플렉싱 불가 |

→ `ApolloCanBus.kt`의 `onHeartbeat`(KY170 `0x07000001` 피드백)가 **실기기에서 절대 안 들어옴.**
조향각 피드백이 없으니 `SteeringActuator` 위치 루프가 닫히지 않음 = 모터 제어 미동작.
**[1] GNSS 부재와 함께 미동작의 2차 핵심 원인.**

수정안(`VanMcu.kt`, 자체 구현):
```kotlin
object VanMcu {
    @JvmStatic external fun CanWrite(channel: Int, id: Int, data: ByteArray): Boolean
    @JvmStatic external fun setCanSpeed(channel: Int, speed: Int): Boolean
    @JvmStatic private external fun setCallback(filterBitmask: Int): Boolean   // Boolean→Int

    private const val FILTER_CAN = 2
    private var canListener: ((Int, Int, ByteArray) -> Unit)? = null

    fun setOnCanListener(cb: (Int, Int, ByteArray) -> Unit) {   // 순수 Kotlin (external 제거)
        canListener = cb
        setCallback(FILTER_CAN)
    }

    @JvmStatic fun onCallback(type: Int, data: ByteArray) {     // ★ native 역호출 — 이름/시그니처 고정
        if (type == FILTER_CAN) {
            val ch = data[0].toInt()
            val id = ((data[1].toInt() and 0xFF) shl 24) or ((data[2].toInt() and 0xFF) shl 16) or
                     ((data[3].toInt() and 0xFF) shl 8) or (data[4].toInt() and 0xFF)   // big-endian
            val dlc = data[5].toInt()
            canListener?.invoke(ch, id, data.copyOfRange(6, 6 + dlc))
        }
    }
}
```
정합 확인된 값(수정 불필요): bitrate `250_000` ✓, TX/RX/HB ID `0x06000001`/`0x05800001`/`0x07000001` ✓.
`ApolloCanBus.start()`는 새 `setOnCanListener((ch,id,data)->…)` 시그니처에 맞춰 수정.
Python `ApolloCanInterface` 스텁도 이 Kotlin 브릿지로 `(channel,id,data)`/`onCallback` 모델 통일.

---

## [4] engage / 주행 제어 흐름

### AGMO 동작 사실
- 제어 루프 진입부는 Qt native(코드 비추출). 데이터 모델 사실만:
- **경로 = RDDF**(`RDDF.java`): `route: LinkedList<LatLng>` + `speeds` + `implement_commands(Boolean)`
  3배열 병렬 → farmmachine `Waypoint(x, y, speed, implement_down, section)`와 1:1 대응.
- **경로 파일 포맷**(`PathManager.parseRddfFromFile`): 탭(`\t`) 구분 텍스트,
  `[1][2]=TM 좌표`(평문 double 아니면 **AES256 복호화**), `[6]=speed`, `[7]=implement("1"=down)`.
  파일명 접두 `AL/AC/ME/FP`=ABLine/ABCurve/Memory/FullPath. S3에 사용자별 저장.
- **좌표계**(`DegreeTMConverter`): 한국 TM(GRS80, 원점경도 **127°E**, false E 200000 / false N 600000).
  위경도↔평면(m) 변환 → 추종은 **평면 m에서 수행**, 표시만 위경도.
- 주행 모드 4종 → farmmachine `ABLineStrategy/ContourStrategy/CustomStrategy`와 매핑
  (Memory=주행기록 재생=Custom, FullPath=전체경로).

### farmmachine 수정 지점
1. **좌표 변환 모듈** — `f9p_client` 뒤단에 위경도→로컬 평면 투영 추가. AGMO 한국 TM 상수를 그대로
   쓸 필요 없음 — **작업 농지 중심 기준 로컬 ENU 투영**으로 자체 구현 권장. 차용 사실은 "위경도가 아닌
   평면 m에서 추종한다"는 점뿐.
2. **제어 주기** — RTK 10Hz/IMU 100Hz 융합. 조향 명령은 100Hz 또는 모터 watchdog과 조화.
   `ApolloCanBus.resetWatchdog`의 800ms keepalive 유지하되 정상 제어 주기는 그보다 짧게.
3. **engage 안전 게이트**(`app_main.py`): `SafetyMonitor`(데드맨 + RTK 4/5 + 속도≤2.5m/s)
   **AND** GNSS heading 유효 **AND** CAN heartbeat 수신 중([3] 수정 후라야 성립). heartbeat 끊기면 즉시 disengage.

---

## 결론 — 미동작 3대 근본 원인 (우선순위)

| # | 근본 원인 | 고칠 파일 | 난이도 |
|---|---|---|---|
| 1 | **GNSS 입력 코드 자체가 없음** — `f9p_client.py`/`app_main.py` 부재, `update_rtk` 호출 0곳 | `f9p_client.py`·`app_main.py`·USB Kotlin 브릿지 (모두 신규) | 中 |
| 2 | **CAN RX 콜백이 .so와 불일치 → heartbeat 영구 미수신** (조향 피드백 루프 안 닫힘) | `android/app/.../can/VanMcu.kt` 시그니처 수정 + `onCallback` static 추가 | 小(확실) |
| 3 | **듀얼안테나 heading 입력 경로 없음** — EKF가 IMU yaw로만 heading 추정 | `autosteer_core.py`(`update_heading`/오프셋), `vendor_profiles.py`(신규) | 中 |

**확정 사실 요약**
- GNSS = **u-blox**(NMEA + UBX; 듀얼안테나 heading = `UBX-NAV-RELPOSNED.relPosHeading`)
- CAN = **`libsysmcu.so` 채널 API**(250k, big-endian id 패킹, `onCallback(int,byte[])` static 역호출,
  필터 비트마스크 CAN=2), socketcan 아님
- 경로 = **RDDF**(LatLng+speed+implement, TM 평면, AES256 옵션, AL/AC/ME/FP)
- 조향 루프·파서 = Qt native (소스 비추출)

**권장 착수 순서**: [3] VanMcu.kt 수정(가장 확실, 작음) → [1] f9p_client.py+app_main.py →
[2] EKF heading 입력. 셋이 모두 들어가야 실기기에서 "위치+heading+조향피드백" 폐루프가 처음으로 닫힌다.
