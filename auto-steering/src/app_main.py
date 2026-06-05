"""
app_main.py
===========
팜머신 자율조향 Android 앱의 Python 진입점 (Chaquopy 임베드).

Kotlin(앱) ↔ Python(알고리즘) 경계:
  Kotlin 은 Python.getModule("app_main") 으로 이 모듈의 모듈레벨 함수를 호출한다.
  - boot(backend="bridge")  앱 시작 시 1회: ApolloCanBus + AutoSteerSystem 기동, 50Hz 루프 시작
  - set_ab_line / set_profile / set_deadman / engage / disengage / estop
  - on_rtk / on_imu          (GNSS·IMU 브릿지가 들어오면 호출; 없으면 안전상 미관여)
  - status_json()            UI 폴링용 상태 JSON
  - shutdown()

CAN 은 apollo_can.ApolloCanBus(backend="bridge") → localhost TCP → Kotlin CAN 서비스.
센서가 안 들어오면 SafetyMonitor 가 RTK_LOW/LOST 로 자동 비활성(안전) 상태 유지.

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
                 hz: float = 50.0):
        if backend == "bridge":
            self.bus = ApolloCanBus(backend="bridge", host=host, port=port,
                                    on_state=lambda s: log.info(f"CAN {s}"))
        else:
            self.bus = ApolloCanBus(backend=backend)
        self.sys = AutoSteerSystem(self.bus, params=KUBOTA_MR1157, algo="implement")
        self.sys.set_profile("normal")
        self._dt = 1.0 / hz
        self._running = False
        self._thread = None
        self._last: dict = {}
        self._lock = threading.Lock()

    # ── 수명주기 ──────────────────────────────────────────────
    def start(self):
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

    # ── 센서 입력 (GNSS/IMU 브릿지에서 호출) ────────────────────
    def on_rtk(self, lat, lon, quality, source="pa3"):
        self.sys.on_rtk(float(lat), float(lon), int(quality), source=str(source))
        self.sys.safety.clear_override()        # 앱에는 운전대 개입 센서 별도

    def on_imu(self, heading, ang_vel, fwd_accel, roll=0.0, pitch=0.0):
        self.sys.on_imu(float(heading), float(ang_vel), float(fwd_accel),
                        self._dt, float(roll), float(pitch))

    # ── 제어 루프 (50Hz) ──────────────────────────────────────
    def _loop(self):
        while self._running:
            t0 = time.time()
            try:
                st = dict(self.sys.control_step(self._dt))
            except Exception as e:
                st = {"error": str(e)}
            s = self.bus.stats
            st.update(running=self._running, can_state=s.state,
                      can_available=self.bus.available, can_tx=s.tx,
                      can_rx=s.rx, can_reconnects=s.reconnects)
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


def boot(backend: str = "bridge", host: str = "127.0.0.1", port: int = 47100):
    global _ctrl
    if _ctrl is None:
        logging.basicConfig(level=logging.INFO)
        _ctrl = Controller(backend=backend, host=host, port=port)
        _ctrl.start()
    return "ok"


def set_ab_line(ax, ay, bx, by, width=3.0, passes=4, speed=1.2):
    return _ctrl.set_ab_line(ax, ay, bx, by, width, passes, speed) if _ctrl else "no-ctrl"


def set_profile(name):  return _ctrl.set_profile(name) if _ctrl else str(name)
def set_deadman(p):     _ctrl and _ctrl.set_deadman(p)
def engage():           return _ctrl.engage() if _ctrl else False
def disengage():        _ctrl and _ctrl.disengage()
def estop():            _ctrl and _ctrl.estop()
def on_rtk(lat, lon, quality, source="pa3"):  _ctrl and _ctrl.on_rtk(lat, lon, quality, source)
def on_imu(h, av, acc, roll=0.0, pitch=0.0):  _ctrl and _ctrl.on_imu(h, av, acc, roll, pitch)
def status_json():      return _ctrl.status_json() if _ctrl else "{}"


def shutdown():
    global _ctrl
    if _ctrl:
        _ctrl.stop()
        _ctrl = None
    return "ok"


# ── CPython 검증 (백엔드=mock, 하드웨어/안드로이드 불필요) ─────────────
if __name__ == "__main__":
    import math
    print("=" * 66)
    print("app_main — Chaquopy 진입점 CPython 검증 (backend=mock)")
    print("=" * 66)

    boot(backend="mock")
    set_ab_line(0, 0, 0, 40, width=3.0, passes=2, speed=1.2)
    set_profile("heavy")
    set_deadman(True)

    # GNSS/IMU 공급 (앱에서는 브릿지가 담당) — 0.6초간 직선주행 시늉
    lat = 37.0
    for k in range(30):
        on_rtk(lat, 127.0, 4, "pa3")
        on_imu(math.pi / 2, 0.0, 0.5)
        lat += 1.2 * 0.02 / 111320
        if k == 5:
            print("engage():", engage())
        time.sleep(0.02)

    st = json.loads(status_json())
    print("\nstatus 일부:")
    for key in ("engaged", "safety", "profile", "can_state", "can_available",
                "can_tx", "can_rx", "xte_cm", "target_angle_deg"):
        print(f"  {key:>16}: {st.get(key)}")

    assert st.get("can_available") is True, "CAN(mock) 미연결"
    assert st.get("can_tx", 0) >= 1, "모터 명령 송신 없음"
    assert "safety" in st and "profile" in st
    estop()
    time.sleep(0.05)
    assert json.loads(status_json()).get("engaged") is False
    shutdown()
    print("\n  ✓ boot→경로설정→engage→제어루프→상태폴링→estop→shutdown 전 경로 동작")
    print("  실기기: boot(backend='bridge') + Kotlin ApolloCanBridge 기동")
