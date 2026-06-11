"""
app_main.py
===========
팜머신 자율조향 Android 앱의 Python 진입점 (Chaquopy 임베드).

Kotlin(앱) ↔ Python(알고리즘) 경계:
  Kotlin 은 Python.getModule("app_main") 으로 이 모듈의 모듈레벨 함수를 호출한다.
  - boot(backend="bridge")  앱 시작 시 1회: 50Hz 제어 루프 시작
  - set_ab_line / set_profile / set_deadman / engage / disengage / estop
  - on_rtk / on_imu          (GNSS·IMU 브릿지가 들어오면 호출)
  - status_json()            UI 폴링용 상태 JSON
  - shutdown()

백엔드 모드:
  - "bridge" : 실기기. ApolloCanBus(localhost TCP) ↔ Kotlin CAN 서비스.
               센서(on_rtk/on_imu)가 안 들어오면 SafetyMonitor 가 자동 비활성(안전).
  - "mock"   : 데모. sitl_sim 으로 자전거모델 폐루프를 돌려 **진짜 알고리즘이**
               가상 RTK/IMU 를 받아 AB라인을 추종 → UI 가 실제 제어값으로 살아 움직인다.

CPython 에서도 backend="mock" 으로 그대로 구동/검증된다(아래 __main__).
"""

from __future__ import annotations
import json
import math
import os
import threading
import time
import logging

from autosteer_core import AutoSteerSystem, KUBOTA_MR1157, ABLineStrategy
from apollo_can import ApolloCanBus

log = logging.getLogger("app_main")


class Controller:
    def __init__(self, backend: str = "bridge",
                 host: str = "127.0.0.1", port: int = 47100,
                 hz: float = 50.0, vendor: str = None,
                 config_dir: str = None):
        self._dt = 1.0 / hz
        self._config_dir = config_dir   # 차량변수 영속화 디렉토리(Android filesDir)
        self.demo = (backend == "mock")
        self.bus = None
        self.sim = None

        if self.demo:
            # 데모: SITL 폐루프(자전거모델 + 가상 RTK/IMU). 실제 알고리즘이 UI 를 구동.
            import sitl_sim
            self.sys = sitl_sim.build_system(algo="pure_pursuit", profile="normal",
                                             realistic=True)
            self.sim = sitl_sim.Simulator(self.sys, KUBOTA_MR1157,
                                          target_speed=1.2, yaw_tau=0.25)
        else:
            self.bus = ApolloCanBus(backend="bridge", host=host, port=port,
                                    on_state=lambda s: log.info(f"CAN {s}"))
            # ★ 기본 = pure_pursuit (안정·SITL 수렴 검증). implement 는 후방참조점 발산
            #   이슈로 재설계 전까지 비활성(set_algorithm("implement")로 수동 선택은 가능).
            self.sys = AutoSteerSystem(self.bus, params=KUBOTA_MR1157, algo="pure_pursuit")
            self.sys.set_profile("normal")

        self._running = False
        self._thread = None
        self._last: dict = {}
        self._lock = threading.Lock()
        self._jog_on = False          # 모터 조그 활성 여부(Enable 시퀀스 추적)
        self._ntrip = None            # NTRIP 클라이언트(RTK 보정신호)
        self._gnss_client = None      # GNSS NMEA 클라이언트(F9P/PA-3) — NTRIP RTCM 주입 대상
        self._hcal = None             # 헤딩 바이어스 캘리브레이터(ver1, 진행 중일 때만)
        self._mdiag = None            # 듀얼 마운트(base/rover·부호) 진단기(진행 중일 때만)
        self._imu_cal = None          # IMU 영점 캘리브레이터(진행 중일 때만)
        self._imu_cal_applied = False
        self._sr_cal = None           # 조향비(steer_ratio) 추정기(진행 중일 때만)
        self._sr_applied = False
        self._sections = 4            # 작업 섹션 수(표시)
        self._ab_a = None             # ⑥ 현장에서 찍은 AB 라인 A점(east,north)
        self._ab_b = None             # ⑥ B점
        self._gnss_job = {"op": None, "running": False, "result": None}  # 비동기 GNSS 작업

        self._load_params()           # 저장된 차량변수 있으면 먼저 반영(휠베이스→알고리즘)
        if vendor:
            self.set_vendor(vendor)

    def set_vendor(self, key: str) -> str:
        """제조사 선택 → 모터 CAN/GNSS/알고리즘 활성화. UI 시작화면에서 호출."""
        p = self.sys.select_vendor(key)
        if self.demo:
            # 데모(SITL)는 시뮬레이터라 모터를 항상 구동(미확정 벤더도 시각화 허용)
            self.sys.motor_verified = True
            self.sys.actuator.speed_control = False   # 데모는 byte-layout 시뮬 유지
        else:
            # 실차 + 프로토콜 확정 벤더(Keya) → 속도제어(cmd_speed) 자율조향 경로 사용.
            # (autosteer engage 는 RTK Fix 필요 + 하트비트 없으면 명령 금지 = 폭주 방지)
            self.sys.actuator.speed_control = bool(self.sys.motor_verified)
        self.sys.set_profile("normal")
        return p.key

    # ── 수명주기 ──────────────────────────────────────────────
    def start(self):
        if self.bus is not None:
            # CAN 트랜시버 전원 먼저(채널 불확실 → 0/1/2 모두 enable, best-effort) → bus 연결
            for ch in (0, 1, 2):
                self.can_power_on(ch)
            self.bus.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="autosteer-loop")
        self._thread.start()
        return "ok"

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.bus is not None:
            self.bus.stop()
        return "ok"

    # ── 설정/명령 ─────────────────────────────────────────────
    def set_ab_line(self, ax, ay, bx, by, width=3.0, passes=4, speed=1.2):
        self.sys.set_path(ABLineStrategy((float(ax), float(ay)),
                                         (float(bx), float(by)),
                                         float(width), int(passes), float(speed)))
        return "ok"

    def set_profile(self, name):
        self.sys.set_profile(str(name))
        return str(name)

    def set_algorithm(self, name):
        """추종 알고리즘 전환: pure_pursuit / stanley / implement_ff(작업기 곡선보정) / implement."""
        self.sys.set_algorithm(str(name), self.sys.params.wheelbase)
        return str(name)

    def set_deadman(self, pressed):
        self.sys.safety.update_deadman(bool(pressed))

    def engage(self):
        return bool(self.sys.engage())

    def disengage(self):
        self.sys.disengage()

    def estop(self):
        self.sys.emergency_stop()
        self.motor_jog(0)

    # ── 모터 점검(조그) — 안테나 없이 모터 구동 확인용 ──────────────
    #   안전: 저속 캡(±JOG_MAX_PERMILLE), hold-to-run(UI가 누르는 동안 반복 호출),
    #   떼면 0 → 정지+Disable. UI가 멈추면 Keya 워치독(1s)이 자동 정지.
    JOG_MAX_PERMILLE = 150        # ≈12 RPM (정격 80RPM 의 15%) — 점검용 저속

    def motor_jog(self, permille):
        """모터 조그: permille(±1000‰)=속도. 0이면 정지. bridge 모드에서만 실동작."""
        from autosteer_core import CanSpec
        p = int(permille)
        p = max(-self.JOG_MAX_PERMILLE, min(self.JOG_MAX_PERMILLE, p))
        if self.bus is None:                 # 데모(mock): 실 CAN 없음
            return "demo"
        if not getattr(self.sys, "motor_verified", True):
            return "blocked"                 # 미확정 벤더 → 출력 차단
        try:
            if p != 0:
                if not self._jog_on:
                    self.bus.send(CanSpec.MOTOR_CMD_ID, CanSpec.CMD_ENABLE)
                    self._jog_on = True
                self.bus.send(CanSpec.MOTOR_CMD_ID, CanSpec.cmd_speed(p))
            else:
                self.bus.send(CanSpec.MOTOR_CMD_ID, CanSpec.cmd_speed(0))
                self.bus.send(CanSpec.MOTOR_CMD_ID, CanSpec.CMD_DISABLE)
                self._jog_on = False
            return "ok"
        except Exception as e:
            log.warning(f"motor_jog 실패: {e}")
            return "error"

    # ── NTRIP (RTK 보정신호) ───────────────────────────────────
    def ntrip_connect(self, host, port, mount, user="", pw=""):
        """NTRIP caster 접속. 받은 RTCM 은 (연결돼 있으면) GNSS 수신기로 전달."""
        import ntrip_client
        self.ntrip_disconnect()
        def on_rtcm(data):
            c = getattr(self, "_gnss_client", None)   # 안테나/F9P 연결 시 보정 주입
            if c is not None and hasattr(c, "write_rtcm"):
                try: c.write_rtcm(data)
                except Exception: pass
        try:
            self._ntrip = ntrip_client.NtripClient(
                str(host), int(port), str(mount), str(user), str(pw), on_rtcm=on_rtcm)
            self._ntrip.start()
            log.info(f"NTRIP 접속 시도: {host}:{port}/{mount}")
            # ⑤ GNSS 가 아직 안 시작됐으면 RTCM 을 흘려보낼 대상(_gnss_client)이 없어
            #    보정이 수신기에 전달되지 않는다 → 사용자에게 순서 경고.
            if getattr(self, "_gnss_client", None) is None:
                return "ok-gnss먼저"   # caster 접속됨, 단 GNSS 시작 전이라 RTCM 미전달
            return "ok"
        except Exception as e:
            log.warning(f"ntrip_connect 실패: {e}")
            return f"error:{e}"

    def ntrip_disconnect(self):
        if self._ntrip:
            self._ntrip.stop(); self._ntrip = None
        return "ok"

    def ntrip_status(self):
        if self._ntrip:
            return self._ntrip.status()
        return {"connected": False, "bytes": 0, "error": "", "host": "", "port": 0, "mount": ""}

    def motor_center(self):
        """현재 모터 누적각을 직진(중앙) 기준으로 캘리브레이션 (WAS 미사용 모드)."""
        if self.bus is not None and hasattr(self.sys, "actuator"):
            self.sys.actuator.set_motor_center(); return "ok"
        return "demo"

    def nudge(self, cm):
        """경로를 좌(+)/우(-)로 cm 만큼 횡이동(넛지)."""
        try:
            self.sys.nudge_path(float(cm) / 100.0); return "ok"
        except Exception as e:
            log.warning(f"nudge 실패: {e}"); return "error"

    def set_section_count(self, n):
        """작업 섹션 수 표시값(저장만 — 추후 섹션 제어 연동)."""
        self._sections = max(1, int(n)); return str(self._sections)

    def set_wheelbase(self, m):
        """휠베이스(m) 변경 + 추종기 재구성."""
        try:
            self.sys.params.wheelbase = float(m)
            # 현재 알고리즘 보존(implement 강제 금지 — 발산 이슈)
            self.sys.set_algorithm(getattr(self.sys, "_algo", "pure_pursuit"),
                                   self.sys.params.wheelbase)
            self._save_params()
            return "ok"
        except Exception as e:
            log.warning(f"set_wheelbase 실패: {e}"); return "error"

    # ── 차량 변수 (실측값) 입력/조회/영속화 ───────────────────────
    PARAM_KEYS = ("wheelbase", "antenna_height", "antenna_to_axle",
                  "antenna_to_impl", "hitch_to_impl", "front_track_width",
                  "max_was_deg")

    def get_params(self):
        """현재 TractorParams(차량 변수) JSON."""
        p = self.sys.params
        return json.dumps({k: getattr(p, k, None) for k in self.PARAM_KEYS},
                          ensure_ascii=False)

    def set_vehicle_params(self, wheelbase, antenna_height,
                           antenna_to_axle, antenna_to_impl):
        """UI 입력(휠베이스·안테나높이·안테나↔뒤차축·안테나↔작업기) → 즉시 반영+저장."""
        p = self.sys.params
        for name, val in (("wheelbase", wheelbase),
                          ("antenna_height", antenna_height),
                          ("antenna_to_axle", antenna_to_axle),
                          ("antenna_to_impl", antenna_to_impl)):
            try:
                if val is not None:
                    setattr(p, name, float(val))
            except Exception as e:
                log.warning(f"파라미터 {name} 적용 실패: {e}")
        # 휠베이스는 조향 기하에 직접 영향 → 알고리즘 재구성
        try:
            self.sys.set_algorithm(getattr(self.sys, "_algo", "pure_pursuit"),
                                   p.wheelbase)
        except Exception as e:
            log.warning(f"알고리즘 재구성 실패: {e}")
        self._save_params()
        log.info(f"차량 변수 적용: wb={p.wheelbase} ah={p.antenna_height} "
                 f"aa={p.antenna_to_axle} ai={p.antenna_to_impl}")
        return self.get_params()

    def _params_path(self):
        if not self._config_dir:
            return None
        return os.path.join(self._config_dir, "tractor_params.json")

    def _save_params(self):
        path = self._params_path()
        if not path:
            return
        try:
            p = self.sys.params
            d = {k: getattr(p, k, None) for k in self.PARAM_KEYS}
            io = getattr(p, "imu_offset", None)   # 파라미터5: IMU 영점(roll/pitch/yaw)
            if io is not None:
                d["imu_offset"] = {"roll": io.roll, "pitch": io.pitch, "yaw": io.yaw}
            sr = getattr(self.sys.actuator, "steer_ratio", None)  # 측정된 조향비
            if sr is not None:
                d["steer_ratio"] = sr
            with open(path, "w") as f:
                json.dump(d, f)
        except Exception as e:
            log.warning(f"차량변수 저장 실패: {e}")

    def _load_params(self):
        path = self._params_path()
        if not path or not os.path.exists(path):
            return
        try:
            with open(path) as f:
                d = json.load(f)
            for k, v in d.items():
                if k in self.PARAM_KEYS and v is not None and hasattr(self.sys.params, k):
                    setattr(self.sys.params, k, float(v))
            io = d.get("imu_offset")
            if io and hasattr(self.sys.params, "imu_offset"):
                from autosteer_core import ImuOffset
                self.sys.params.imu_offset = ImuOffset(
                    roll=float(io.get("roll", 0.0)),
                    pitch=float(io.get("pitch", 0.0)),
                    yaw=float(io.get("yaw", 0.0)))
            if d.get("steer_ratio") and hasattr(self.sys, "actuator"):
                self.sys.actuator.steer_ratio = float(d["steer_ratio"])
            log.info(f"저장된 차량변수 로드: {d}")
        except Exception as e:
            log.warning(f"차량변수 로드 실패: {e}")

    # ── 센서 입력 (bridge 모드에서 GNSS/IMU 브릿지가 호출) ───────
    def on_rtk(self, lat, lon, quality, source="pa3"):
        if self.demo:
            return                      # 데모는 SITL 이 RTK 를 공급
        self.sys.on_rtk(float(lat), float(lon), int(quality), source=str(source))
        self.sys.safety.clear_override()

    def on_imu(self, heading, ang_vel, fwd_accel, roll=0.0, pitch=0.0):
        if self.demo:
            return
        self.sys.on_imu(float(heading), float(ang_vel), float(fwd_accel),
                        self._dt, float(roll), float(pitch))
        self._imu_cal_feed(roll, pitch, math.radians(float(heading)))

    def on_gyro(self, ang_vel, fwd_accel=0.0, roll=0.0, pitch=0.0):
        """IMU 각속도(yaw rate, rad/s) — 듀얼안테나 절대heading 과 융합(스네이크 억제).
        ★ 태블릿 SensorManager 는 장착방향 미지 → opt-in(ImuBridge 기본 off). 도메 IMU 선호."""
        if self.demo:
            return
        self.sys.on_gyro(float(ang_vel), float(fwd_accel), self._dt,
                         float(roll), float(pitch))

    def on_accel(self, ay, az, roll_rate=0.0, lin_acc=0.0, yaw_rate=0.0):
        """IMU 가속도(roll 보완필터, 경사 보정). 원심오염 배제는 코어에서 처리."""
        if self.demo:
            return
        self.sys.on_accel(float(ay), float(az), float(roll_rate), self._dt,
                          float(lin_acc), float(yaw_rate))
        # 가속도만 있는 IMU: 평지 정차 roll = atan2(ay, az) (pitch 는 ax 없어 0)
        self._imu_cal_feed(math.atan2(float(ay), float(az)), 0.0, 0.0)

    # ── ⑥ 현장 AB 라인: 현재 GNSS 위치를 A/B 로 마킹 → 평행 패스 생성 ──
    def mark_ab(self, which):
        """현재 차량(EKF) 위치를 A('a') 또는 B('b') 점으로 기록."""
        st = self.sys.estimator.get_state()
        pt = (round(st.x, 3), round(st.y, 3))
        if str(which).lower().startswith("a"):
            self._ab_a = pt
        else:
            self._ab_b = pt
        return json.dumps({"a": self._ab_a, "b": self._ab_b}, ensure_ascii=False)

    def build_ab(self, width=3.0, passes=4, speed=1.2):
        """찍어둔 A·B 로 AB 직선 경로 생성. A/B 미설정·너무 가까우면 거부."""
        if not self._ab_a or not self._ab_b:
            return "need-ab"
        if math.hypot(self._ab_b[0]-self._ab_a[0],
                      self._ab_b[1]-self._ab_a[1]) < 1.0:
            return "too-short"   # A↔B ≥ 1m 필요(방향 산출)
        self.set_ab_line(self._ab_a[0], self._ab_a[1],
                         self._ab_b[0], self._ab_b[1],
                         float(width), int(passes), float(speed))
        return "ok"

    def ab_status(self):
        return json.dumps({"a": self._ab_a, "b": self._ab_b}, ensure_ascii=False)

    def on_heading(self, compass_deg):
        """INS/듀얼안테나 진헤딩(나침반°). Kotlin GNSS 브릿지가 직접 푸시할 때."""
        if self.demo:
            return
        compass_deg = float(compass_deg)
        self.sys.on_heading(compass_deg)
        # ver1 헤딩 바이어스 캘리브 진행 중이면 (위치,보고헤딩) 샘플 수집
        if self._hcal is not None:
            st = self.sys.estimator.get_state()
            self._hcal.add_sample(st.x, st.y, compass_deg)
            if self._hcal.ready():
                est = self._hcal.finish()
                if est.ok:
                    self.sys.set_heading_bias(est.value)
                log.info(f"헤딩 캘리브 완료: {est.note}")
                self._hcal = None

    def start_heading_calib(self):
        """ver1 듀얼안테나 헤딩 바이어스 캘리브 시작 — 이후 ~20m 직선 주행하면 자동 산출/적용."""
        from calibration import HeadingCalibrator
        self._hcal = HeadingCalibrator()
        return "ok"

    def start_heading_calib_drive(self, length_m=20.0, width=3.0, speed=1.0):
        """★ 헤딩 캘리브용 '전방 직선 자동생성' — 현재 위치/헤딩에서 length_m 앞으로 AB라인
        생성 + 헤딩캘리브 동시 시작. 이후 engage(데드맨)로 그 직선을 autosteer 가 따라가면
        사람 수동주행보다 곧게 달려 COG(진로각) 기준이 깨끗 → 헤딩 바이어스가 더 정확.
        (듀얼안테나 절대헤딩이 처음부터 있어 bootstrap 문제 없음.)"""
        st = self.sys.estimator.get_state()
        h = st.heading
        ax, ay = st.x, st.y
        bx = ax + float(length_m) * math.cos(h)
        by = ay + float(length_m) * math.sin(h)
        self.set_ab_line(ax, ay, bx, by, float(width), 1, float(speed))
        self.start_heading_calib()
        log.info(f"헤딩 캘리브 자동직선 생성: ({ax:.1f},{ay:.1f})→({bx:.1f},{by:.1f}) {length_m}m")
        return json.dumps({"a": [round(ax, 2), round(ay, 2)],
                           "b": [round(bx, 2), round(by, 2)], "len": length_m},
                          ensure_ascii=False)

    def heading_calib_status(self):
        if self._hcal is None:
            return {"active": False, "progress": 1.0, "bias_deg": self.sys.heading_bias_deg if not self.demo else 0.0}
        return {"active": True, "progress": round(self._hcal.progress, 3),
                "bias_deg": self.sys.heading_bias_deg if not self.demo else 0.0}

    # ── IMU 영점 캘리브 (파라미터5) — 평지 정차 30초 평균 → ImuOffset ──
    #   ★ 배선만 미리 완성. IMU 데이터(on_imu roll/pitch 또는 on_accel ay/az)가
    #   들어와야 실제로 진행됨. 이 기기(ApolloPro) IMU 읽기 경로 확정 후 데이터 유입.
    def start_imu_calib(self):
        """평지 정차 IMU 영점 캘리브 시작. 이후 on_imu/on_accel 샘플로 자동 누적·적용."""
        from autosteer_core import ImuCalibrator
        self._imu_cal = ImuCalibrator()
        self._imu_cal.start()
        self._imu_cal_applied = False
        return "ok"

    def _imu_cal_feed(self, roll, pitch, yaw=0.0):
        """진행 중이면 IMU 샘플 누적. ready 시 finish→params.imu_offset 적용·저장."""
        c = self._imu_cal
        if c is None:
            return
        try:
            c.add_sample(float(roll), float(pitch), float(yaw))
        except Exception:
            return
        if c.ready:
            off = c.finish()
            self.sys.params.imu_offset = off
            self._save_params()
            self._imu_cal = None
            self._imu_cal_applied = True
            log.info(f"IMU 영점 적용·저장: roll={off.roll:+.4f} pitch={off.pitch:+.4f}")

    # ── 조향비(steer_ratio) 자동측정 — S자/사인 주행으로 17.5 가정 대체 ──
    #   δ=atan(ω·L/v) 역산 vs 모터 하트비트각 회귀 = steer_ratio. wheelbase(L) 기지 전제.
    #   ★ 모터 하트비트(RX) + GNSS yaw 필요. 좌우 조향 변화가 있어야 추정됨(직선만은 불가).
    def start_steer_ratio_calib(self):
        from calibration import SteerRatioEstimator
        self._sr_cal = SteerRatioEstimator(wheelbase=self.sys.params.wheelbase)
        self._sr_applied = False
        return "ok"

    def _sr_feed(self):
        c = self._sr_cal
        if c is None:
            return
        ma = self.sys.actuator.get_motor_angle_rad()   # 하트비트 없으면 None
        if ma is None:
            return
        st = self.sys.estimator.get_state()
        c.add_sample(ma, st.speed, st.angular_vel)
        est = c.estimate()
        if est.ok and est.n_samples >= 100:
            self.sys.actuator.steer_ratio = est.value
            self._save_params()
            self._sr_cal = None
            self._sr_applied = True
            log.info(f"조향비 측정·적용: steer_ratio={est.value:.2f} (n={est.n_samples}, R²={est.r2:.3f})")

    def steer_ratio_calib_status(self):
        sr = getattr(self.sys.actuator, "steer_ratio", 17.5)
        c = self._sr_cal
        if c is None:
            return {"active": False, "steer_ratio": sr, "applied": self._sr_applied}
        est = c.estimate()
        return {"active": True, "n": est.n_samples, "r2": round(est.r2, 3),
                "steer_ratio": sr, "applied": False}

    def imu_calib_status(self):
        p = getattr(self.sys.params, "imu_offset", None)
        base = {"roll": p.roll if p else 0.0, "pitch": p.pitch if p else 0.0,
                "yaw": p.yaw if p else 0.0}
        c = self._imu_cal
        if c is None:
            base.update(active=False, progress=1.0 if self._imu_cal_applied else 0.0,
                        applied=self._imu_cal_applied)
        else:
            base.update(active=True, progress=round(c.progress, 3),
                        samples=len(c._roll), applied=False)
        return base

    def _on_heading_meas(self, meas):
        """무빙베이스 RELPOSNED → EKF 반영 + (진행 중이면) 듀얼 마운트 진단 샘플 수집."""
        self.sys.on_heading_meas(meas)
        if self._mdiag is not None:
            st = self.sys.estimator.get_state()      # x=east, y=north
            self._mdiag.add_sample(st.x, st.y,
                                   float(meas.get("heading_deg", 0.0)),
                                   float(meas.get("rel_d_m", 0.0)),
                                   float(meas.get("baseline_m", 1.0)),
                                   float(meas.get("acc_deg", 0.0)))
            self._mdiag.maybe_log()

    def start_mount_diag(self):
        """듀얼안테나 base/rover·부호 진단 시작 — 이후 직선 ~15m 주행(끝에 한쪽으로 살짝 기울이면 roll 부호까지)."""
        if self.demo:
            return "demo"
        from calibration import DualMountDiagnostic
        self._mdiag = DualMountDiagnostic()
        return "ok"

    def mount_diag_status(self):
        if self._mdiag is None:
            return {"active": False}
        r = self._mdiag.report(); r["active"] = True
        return r

    # ★ Apollo2(RK3568) 하드웨어 전원 GPIO — 디컴파일 동작 사실(GPIO 번호만, clean-room).
    #   GNSS = Unicore UM482 듀얼안테나 헤딩 보드(u-blox 아님). 표준 Linux sysfs
    #   /sys/class/gpio/gpioNN/value 로 1 을 써서 켠다. 켜는 순서: 전원 → LNA → 리셋해제.
    #   권한(시스템/root) 없으면 best-effort 실패 → logcat. (AGMO 는 별도 시스템서비스가 켬)
    GPIO_UM482_PWREN  = 137   # GNSS(UM482) 보드 전원
    GPIO_GNSS_LNA_EN  = 101   # 안테나 LNA 전원
    GPIO_GNSS_RST_N   = 136   # GNSS 리셋(액티브 로우 → 1=해제)
    GPIO_CAN_PWR_EN   = 61    # CAN 트랜시버 전원(공통)
    GPIO_CAN_ON       = {0: 99, 1: 154, 2: 128}   # 채널별 CANx_ON enable
    GPIO_RS485_EN     = 134   # RS-485(LoRa NTRIP) 전원

    @staticmethod
    def _gpio_set(num, value=1):
        """RK3568 sysfs GPIO 출력. export→direction(out)→value. best-effort."""
        base = f"/sys/class/gpio/gpio{num}"
        try:
            if not os.path.exists(base):
                try:
                    with open("/sys/class/gpio/export", "w") as f:
                        f.write(str(num))
                except Exception:
                    pass
            try:
                with open(os.path.join(base, "direction"), "w") as f:
                    f.write("out")
            except Exception:
                pass
            with open(os.path.join(base, "value"), "w") as f:
                f.write("1" if value else "0")
            return True
        except Exception as e:
            log.info(f"GPIO{num} 설정 불가: {e} — 권한(시스템/root)·플랫폼 현장 확인")
            return False

    def scan_gnss(self, window=1.5):
        """
        내부 UART 자동 스니핑 — 어느 /dev/ttySx 에 GNSS(NMEA/UBX)가 나오는지 탐지(1단계).
        ⚠ 블로킹(포트×보레이트×window). 전원 먼저 켠다.
        """
        if self.demo:
            return {"demo": True}
        self.gnss_power_on()
        import f9p_client as fc
        return fc.scan_ports(window=float(window))

    def configure_moving_base(self, port="/dev/ttyHSL0", baud=0):
        """
        u-blox 무빙베이스 듀얼안테나 heading 출력(UBX-NAV-RELPOSNED)+위치/속도 활성·저장.
        ★ 전제: 수신기가 무빙베이스/로버 모드(AGMO 돔=공장설정 추정). 적용 후 RELPOSNED 가
          안 나오면 모드 미설정 → 현장 조사. (f9p_client.moving_base_heading_cfg 주석 참고)
        """
        if self.demo:
            return "demo"
        import f9p_client as fc
        spec = getattr(self.sys.vendor, "gnss_primary", None) if self.sys.vendor else None
        baud = int(baud) or (spec.serial_baud if spec else 115200)
        return fc.configure_serial(str(port), baud, fc.moving_base_heading_cfg())

    # ★ ApolloPro(이 실기기, Qualcomm) GNSS = u-blox. 전원은 sysfs gpio 가 아니라
    #   /dev/gpio_dev 매직코드(디컴파일 ApolloPro.setUblox/ setRs485 확정). 666 → 일반앱 가능.
    GPIO_DEV       = "/dev/gpio_dev"
    GPIO_DEV_UBLOX_ON  = "100008"   # setUblox(1)
    GPIO_DEV_UBLOX_OFF = "100009"
    GPIO_DEV_RS485_ON  = "100021"   # setRs485(1) — NTRIP/RTCM 보정 라인

    @classmethod
    def _gpio_dev_write(cls, code):
        """ApolloPro /dev/gpio_dev 매직코드 echo (best-effort, 666이면 일반앱도 됨)."""
        try:
            with open(cls.GPIO_DEV, "w") as f:
                f.write(str(code) + "\n")
            return True
        except Exception as e:
            log.info(f"gpio_dev {code} 쓰기 실패: {e}")
            return False

    def gnss_power_on(self):
        """GNSS 전원 ON. ApolloPro(u-blox, /dev/gpio_dev 100008)가 이 실기기 경로.
        Apollo2(RK3568 sysfs UM482) 는 폴백으로 함께 시도(멀티변종)."""
        if self.demo:
            return "demo"
        ok = []
        # 1) ApolloPro (이 기기): /dev/gpio_dev 매직코드 — u-blox ON + RS485(NTRIP) ON
        if self._gpio_dev_write(self.GPIO_DEV_UBLOX_ON): ok.append("ublox(100008)")
        if self._gpio_dev_write(self.GPIO_DEV_RS485_ON): ok.append("rs485(100021)")
        # 2) Apollo2 (RK3568) 폴백: sysfs gpio
        if self._gpio_set(self.GPIO_UM482_PWREN, 1): ok.append("PWREN")
        if self._gpio_set(self.GPIO_GNSS_LNA_EN, 1): ok.append("LNA")
        if self._gpio_set(self.GPIO_GNSS_RST_N, 1):  ok.append("RST_N")
        if ok:
            log.info(f"GNSS 전원 ON: {ok}")
            return "ok"
        log.info("GNSS 전원 쓰기 실패 — 권한 또는 경로 확인")
        return "no-gpio"

    def can_power_on(self, channel=0):
        """CAN 트랜시버 전원 ON — CAN_PWR_EN(61) + 채널 CANx_ON. CanWrite 전 필요."""
        if self.demo:
            return "demo"
        ok = []
        if self._gpio_set(self.GPIO_CAN_PWR_EN, 1): ok.append("PWR_EN")
        ch_gpio = self.GPIO_CAN_ON.get(int(channel))
        if ch_gpio is not None and self._gpio_set(ch_gpio, 1): ok.append(f"CAN{channel}_ON")
        if ok:
            log.info(f"CAN 전원 GPIO ON: {ok}")
            return "ok"
        return "no-gpio"

    def start_gnss(self, port="/dev/ttyHSL0", baud=0):
        """
        GNSS(NMEA GGA 위치 + HDT/UBX 진헤딩) 읽어 on_rtk/on_heading(_meas) 공급.
        ★ 자율조향 1단계 = AGMO ver1 안테나(내부 UART). port 는 내부 tty(예: /dev/ttyS1~S3) —
          실기기에서 어느 포트에 NMEA 가 나오는지 현장 스니핑으로 확정. (USB 아님; USB=레벨러 전용)
        baud=0 이면 벤더 GNSS 스펙 serial_baud. 포트/안테나 없으면 안전 무동작.
        PA-3/NX510(CAN/RS232)은 실험 후 추가.
        """
        if self.demo:
            return "demo"
        self.gnss_power_on()       # 내부 u-blox 전원 ON 시도(없으면 무시)
        import f9p_client as fc
        spec = getattr(self.sys.vendor, "gnss_primary", None) if self.sys.vendor else None
        baud = int(baud) or (spec.serial_baud if spec else 115200)
        src = self.sys.gnss.primary
        self._gnss_client = fc.F9pUsbClient(
            port=str(port), baudrate=baud, source=src,
            on_rtk=lambda la, lo, q: self.sys.on_rtk(la, lo, q, source=src),
            on_heading=self.sys.on_heading,
            on_heading_meas=self._on_heading_meas,      # 무빙베이스 RELPOSNED(방법 1·3·4) + 마운트 진단
            on_velocity=self.sys.on_velocity,           # 진로각 보조(방법 5)
            on_gyro=self.on_gyro,                       # 안테나 IMU yaw rate(UBX-ESF-MEAS) → 듀얼heading 융합
            on_accel=lambda ay, az: self.on_accel(ay, az))  # 안테나 IMU 가속도 → roll 보정
        ok = False
        try:
            ok = self._gnss_client.start()
        except Exception as e:
            log.warning(f"GNSS 시작 실패: {e}")
        if not ok:
            log.info("GNSS 미연결(안테나/포트 없음) — 연결되면 재시도/자동 동작")
        return "ok" if ok else "no-gnss"

    # ── 비동기 GNSS 작업 (포트탐지/설정/시작은 블로킹 → UI 프리즈 방지) ──
    #   scan_ports 는 포트×보레이트×window 로 수십 초 블로킹한다. JsBridge 호출은
    #   WebView JS 스레드를 막아 화면이 멈추므로, 백그라운드 스레드에서 돌리고
    #   UI 는 gnss_job_status() 를 폴링한다(한 번에 하나, 순차 사용에 적합).
    def _run_async(self, op, fn):
        if self._gnss_job.get("running"):
            return json.dumps({"running": True, "op": self._gnss_job.get("op")})
        self._gnss_job = {"op": op, "running": True, "result": None}

        def _work():
            try:
                res = fn()
            except Exception as e:
                log.warning(f"GNSS 작업 실패({op}): {e}")
                res = {"error": str(e)}
            self._gnss_job = {"op": op, "running": False, "result": res}

        threading.Thread(target=_work, daemon=True, name=f"gnss-{op}").start()
        return json.dumps({"running": True, "op": op})

    def gnss_job_status(self):
        j = self._gnss_job
        return json.dumps({"op": j.get("op"), "running": bool(j.get("running")),
                           "result": j.get("result")}, ensure_ascii=False)

    def scan_gnss_async(self, window=1.5):
        return self._run_async("scan", lambda: self.scan_gnss(window))

    def configure_moving_base_async(self, port="/dev/ttyHSL0", baud=0):
        return self._run_async("configure",
                               lambda: {"result": self.configure_moving_base(port, baud)})

    def start_gnss_async(self, port="/dev/ttyHSL0", baud=0):
        return self._run_async("start",
                               lambda: {"result": self.start_gnss(port, baud)})

    # ── 제어 루프 (50Hz) ──────────────────────────────────────
    def _loop(self):
        while self._running:
            t0 = time.time()
            try:
                if self.demo:
                    st = dict(self.sim.step(self._dt))   # 폐루프 한 틱(rtk/imu 주입+제어)
                else:
                    st = dict(self.sys.control_step(self._dt))
                    if self._sr_cal is not None:
                        self._sr_feed()       # 조향비 추정 샘플 누적(진행 중일 때만)
            except Exception as e:
                st = {"error": str(e)}
            if self.bus is not None:
                s = self.bus.stats
                st.update(can_state=s.state, can_available=self.bus.available,
                          can_tx=s.tx, can_rx=s.rx, can_reconnects=s.reconnects)
                try: st["heartbeat"] = self.sys.actuator.latest_heartbeat()
                except Exception: pass
            else:
                st.update(can_state="SIM", can_available=True,
                          can_tx=0, can_rx=0, can_reconnects=0)
            st["running"] = self._running
            try: st["algo"] = getattr(self.sys, "_algo", "pure_pursuit")
            except Exception: pass
            with self._lock:
                self._last = st
            rest = self._dt - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)

    def status(self) -> dict:
        with self._lock:
            return dict(self._last)

    def status_json(self) -> str:
        return json.dumps(self.status(), ensure_ascii=False)


# ── 모듈레벨 API (Kotlin/Chaquopy 호출 표면) ────────────────────────
_ctrl: "Controller | None" = None


def boot(backend: str = "bridge", config_dir: str = None,
         host: str = "127.0.0.1", port: int = 47100,
         vendor: str = None):
    global _ctrl
    if _ctrl is None:
        logging.basicConfig(level=logging.INFO)
        # ★ 실차(bridge)에서 벤더 미지정이면 오너 기본 스택(AGMO/Keya)으로 부팅.
        #   → select_vendor 가 actuator.speed_control=True(cmd_speed 경로) 활성.
        #   (벤더 미선택 시 speed_control=False → placeholder _send_motor 로 모터 무반응)
        #   UI 시작화면에서 set_vendor 로 런타임 변경 가능.
        if vendor is None and backend == "bridge":
            vendor = "agmo"
        _ctrl = Controller(backend=backend, host=host, port=port, vendor=vendor,
                           config_dir=config_dir)
        _ctrl.start()
    return "ok"


def list_vendors() -> str:
    """제조사 선택화면용 목록 JSON. (부팅 전에도 호출 가능)"""
    import vendor_profiles
    return json.dumps(vendor_profiles.list_vendors(), ensure_ascii=False)


def set_vendor(key: str) -> str:
    """제조사 선택. 부팅 전이면 그 벤더로 부팅, 부팅 후면 런타임 전환."""
    if _ctrl is None:
        boot(vendor=key)
        return key
    return _ctrl.set_vendor(key)


def set_ab_line(ax, ay, bx, by, width=3.0, passes=4, speed=1.2):
    return _ctrl.set_ab_line(ax, ay, bx, by, width, passes, speed) if _ctrl else "no-ctrl"


def set_profile(name):  return _ctrl.set_profile(name) if _ctrl else str(name)
def set_algorithm(name): return _ctrl.set_algorithm(name) if _ctrl else str(name)
def set_deadman(p):     _ctrl and _ctrl.set_deadman(p)
def engage():           return _ctrl.engage() if _ctrl else False
def disengage():        _ctrl and _ctrl.disengage()
def estop():            _ctrl and _ctrl.estop()
def motor_jog(permille): return _ctrl.motor_jog(permille) if _ctrl else "no-ctrl"
def motor_center():      return _ctrl.motor_center() if _ctrl else "no-ctrl"
def nudge(cm):           return _ctrl.nudge(cm) if _ctrl else "no-ctrl"
def set_section_count(n): return _ctrl.set_section_count(n) if _ctrl else str(n)
def set_wheelbase(m):    return _ctrl.set_wheelbase(m) if _ctrl else "no-ctrl"
def get_params():        return _ctrl.get_params() if _ctrl else "{}"
def set_vehicle_params(wheelbase, antenna_height, antenna_to_axle, antenna_to_impl):
    return _ctrl.set_vehicle_params(wheelbase, antenna_height, antenna_to_axle, antenna_to_impl) if _ctrl else "{}"
def ntrip_connect(host, port, mount, user="", pw=""):
    return _ctrl.ntrip_connect(host, port, mount, user, pw) if _ctrl else "no-ctrl"
def ntrip_disconnect(): return _ctrl.ntrip_disconnect() if _ctrl else "no-ctrl"
def ntrip_status():     return json.dumps(_ctrl.ntrip_status() if _ctrl else
                                          {"connected": False, "bytes": 0, "error": ""}, ensure_ascii=False)
def on_heading(compass_deg):  _ctrl and _ctrl.on_heading(compass_deg)
def start_heading_calib():    return _ctrl.start_heading_calib() if _ctrl else "no-ctrl"
def start_heading_calib_drive(length_m=20.0, width=3.0, speed=1.0): return _ctrl.start_heading_calib_drive(length_m, width, speed) if _ctrl else "no-ctrl"
def heading_calib_status():   return json.dumps(_ctrl.heading_calib_status() if _ctrl else {"active": False}, ensure_ascii=False)
def start_imu_calib():        return _ctrl.start_imu_calib() if _ctrl else "no-ctrl"
def imu_calib_status():       return json.dumps(_ctrl.imu_calib_status() if _ctrl else {"active": False, "progress": 0.0}, ensure_ascii=False)
def start_steer_ratio_calib(): return _ctrl.start_steer_ratio_calib() if _ctrl else "no-ctrl"
def steer_ratio_calib_status(): return json.dumps(_ctrl.steer_ratio_calib_status() if _ctrl else {"active": False}, ensure_ascii=False)
def start_mount_diag():       return _ctrl.start_mount_diag() if _ctrl else "no-ctrl"
def mount_diag_status():      return json.dumps(_ctrl.mount_diag_status() if _ctrl else {"active": False}, ensure_ascii=False)
def start_gnss(port="/dev/ttyHSL0", baud=0): return _ctrl.start_gnss(port, baud) if _ctrl else "no-ctrl"
def gnss_power_on():    return _ctrl.gnss_power_on() if _ctrl else "no-ctrl"
def scan_gnss(window=1.5): return json.dumps(_ctrl.scan_gnss(window) if _ctrl else {"best": None, "ports": []}, ensure_ascii=False)
def scan_gnss_async(window=1.5): return _ctrl.scan_gnss_async(window) if _ctrl else '{"running":false}'
def configure_moving_base_async(port="/dev/ttyHSL0", baud=0): return _ctrl.configure_moving_base_async(port, baud) if _ctrl else '{"running":false}'
def start_gnss_async(port="/dev/ttyHSL0", baud=0): return _ctrl.start_gnss_async(port, baud) if _ctrl else '{"running":false}'
def gnss_job_status(): return _ctrl.gnss_job_status() if _ctrl else '{"running":false,"result":null}'
def configure_moving_base(port="/dev/ttyHSL0", baud=0): return _ctrl.configure_moving_base(port, baud) if _ctrl else "no-ctrl"
def on_rtk(lat, lon, quality, source="pa3"):  _ctrl and _ctrl.on_rtk(lat, lon, quality, source)
def on_imu(h, av, acc, roll=0.0, pitch=0.0):  _ctrl and _ctrl.on_imu(h, av, acc, roll, pitch)
def on_gyro(av, acc=0.0, roll=0.0, pitch=0.0):  _ctrl and _ctrl.on_gyro(av, acc, roll, pitch)
def on_accel(ay, az, roll_rate=0.0, lin_acc=0.0, yaw_rate=0.0):  _ctrl and _ctrl.on_accel(ay, az, roll_rate, lin_acc, yaw_rate)
def mark_ab(which):     return _ctrl.mark_ab(which) if _ctrl else "no-ctrl"
def build_ab(width=3.0, passes=4, speed=1.2): return _ctrl.build_ab(width, passes, speed) if _ctrl else "no-ctrl"
def ab_status():        return _ctrl.ab_status() if _ctrl else "{}"
def status_json():      return _ctrl.status_json() if _ctrl else "{}"


def shutdown():
    global _ctrl
    if _ctrl:
        _ctrl.stop()
        _ctrl = None
    return "ok"


# ── CPython 검증 (backend=mock = SITL 데모, 안드로이드/하드웨어 불필요) ──
if __name__ == "__main__":
    print("=" * 66)
    print("app_main — Chaquopy 진입점 CPython 검증 (backend=mock = SITL 데모)")
    print("=" * 66)

    boot(backend="mock")               # SITL 자동 engage + 폐루프 시작
    time.sleep(0.8)                    # 50Hz 루프가 한동안 추종하게 둠

    st = json.loads(status_json())
    print("\nstatus 일부:")
    for key in ("engaged", "safety", "profile", "active_gnss",
                "xte_cm", "target_angle_deg", "measured_angle_deg",
                "speed_mps", "can_state"):
        print(f"  {key:>18}: {st.get(key)}")

    assert st.get("engaged") is True, "SITL 데모가 engage 되지 않음"
    assert st.get("safety") == "SAFE", f"안전상태 비정상: {st.get('safety')}"
    assert "xte_cm" in st and abs(st["xte_cm"]) < 100, "XTE 비정상/발산"
    assert st.get("speed_mps", 0) > 0.3, "속도 미상승"

    print("\n프로파일 전환/해제 테스트:")
    print("  set_profile('heavy') →", set_profile("heavy"))
    print("  disengage()");  disengage(); time.sleep(0.05)
    assert json.loads(status_json()).get("engaged") is False
    print("  engage() →", engage());  time.sleep(0.05)
    assert json.loads(status_json()).get("engaged") is True
    estop(); time.sleep(0.05)
    assert json.loads(status_json()).get("engaged") is False
    shutdown()
    print("\n  ✓ mock=SITL 폐루프: 자동 engage→실제 알고리즘 추종→프로파일/해제/estop 동작")
    print("  실기기: boot(backend='bridge') + Kotlin ApolloCanBridge + GNSS/IMU 브릿지")
