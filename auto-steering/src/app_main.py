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
import threading
import time
import logging

from autosteer_core import AutoSteerSystem, KUBOTA_MR1157, ABLineStrategy
from apollo_can import ApolloCanBus

log = logging.getLogger("app_main")


class Controller:
    def __init__(self, backend: str = "bridge",
                 host: str = "127.0.0.1", port: int = 47100,
                 hz: float = 50.0, vendor: str = None):
        self._dt = 1.0 / hz
        self.demo = (backend == "mock")
        self.bus = None
        self.sim = None

        if self.demo:
            # 데모: SITL 폐루프(자전거모델 + 가상 RTK/IMU). 실제 알고리즘이 UI 를 구동.
            import sitl_sim
            self.sys = sitl_sim.build_system(algo="implement", profile="normal",
                                             realistic=True)
            self.sim = sitl_sim.Simulator(self.sys, KUBOTA_MR1157,
                                          target_speed=1.2, yaw_tau=0.25)
        else:
            self.bus = ApolloCanBus(backend="bridge", host=host, port=port,
                                    on_state=lambda s: log.info(f"CAN {s}"))
            self.sys = AutoSteerSystem(self.bus, params=KUBOTA_MR1157, algo="implement")
            self.sys.set_profile("normal")

        self._running = False
        self._thread = None
        self._last: dict = {}
        self._lock = threading.Lock()
        self._jog_on = False          # 모터 조그 활성 여부(Enable 시퀀스 추적)
        self._ntrip = None            # NTRIP 클라이언트(RTK 보정신호)

        if vendor:
            self.set_vendor(vendor)

    def set_vendor(self, key: str) -> str:
        """제조사 선택 → 모터 CAN/GNSS/알고리즘 활성화. UI 시작화면에서 호출."""
        p = self.sys.select_vendor(key)
        if self.demo:
            # 데모(SITL)는 시뮬레이터라 모터를 항상 구동(미확정 벤더도 시각화 허용)
            self.sys.motor_verified = True
        self.sys.set_profile("normal")
        return p.key

    # ── 수명주기 ──────────────────────────────────────────────
    def start(self):
        if self.bus is not None:
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

    # ── 제어 루프 (50Hz) ──────────────────────────────────────
    def _loop(self):
        while self._running:
            t0 = time.time()
            try:
                if self.demo:
                    st = dict(self.sim.step(self._dt))   # 폐루프 한 틱(rtk/imu 주입+제어)
                else:
                    st = dict(self.sys.control_step(self._dt))
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


def boot(backend: str = "bridge", host: str = "127.0.0.1", port: int = 47100,
         vendor: str = None):
    global _ctrl
    if _ctrl is None:
        logging.basicConfig(level=logging.INFO)
        _ctrl = Controller(backend=backend, host=host, port=port, vendor=vendor)
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
def set_deadman(p):     _ctrl and _ctrl.set_deadman(p)
def engage():           return _ctrl.engage() if _ctrl else False
def disengage():        _ctrl and _ctrl.disengage()
def estop():            _ctrl and _ctrl.estop()
def motor_jog(permille): return _ctrl.motor_jog(permille) if _ctrl else "no-ctrl"
def motor_center():      return _ctrl.motor_center() if _ctrl else "no-ctrl"
def ntrip_connect(host, port, mount, user="", pw=""):
    return _ctrl.ntrip_connect(host, port, mount, user, pw) if _ctrl else "no-ctrl"
def ntrip_disconnect(): return _ctrl.ntrip_disconnect() if _ctrl else "no-ctrl"
def ntrip_status():     return json.dumps(_ctrl.ntrip_status() if _ctrl else
                                          {"connected": False, "bytes": 0, "error": ""}, ensure_ascii=False)
def on_rtk(lat, lon, quality, source="pa3"):  _ctrl and _ctrl.on_rtk(lat, lon, quality, source)
def on_imu(h, av, acc, roll=0.0, pitch=0.0):  _ctrl and _ctrl.on_imu(h, av, acc, roll, pitch)
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
