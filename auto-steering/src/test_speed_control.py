"""
test_speed_control.py — 무WAS Keya 속도제어 조향 폐루프 검증 (시뮬, 의존성 X)

실하드웨어 없이 알고리즘 정합성 확인:
  SteeringActuator(speed_control=True, use_motor_encoder=True)
    → cmd_speed 송신 → (Keya 시뮬: permille→모터각 적분→하트비트)
    → process_can_recv(하트비트→조향각) → 수렴 확인.

검증 항목:
  1) 하트비트 미수신 동안 명령 0 (폭주 방지 안전가드)
  2) 목표 조향각으로 수렴
  3) 부호: target>0 → +permille (규약: +=좌회전)
"""
import math
import autosteer_core as c
from autosteer_core import CanSpec, SteeringActuator, CanInterface


def _decode_value(b: bytes) -> int:
    low = (b[0] << 8) | b[1]
    high = (b[2] << 8) | b[3]
    v = (high << 16) | low
    return v - 0x100000000 if v >= 0x80000000 else v


class KeyaSimCan(CanInterface):
    """cmd_speed 수신 → 모터 속도 → 누적각 적분 → 하트비트 송신."""
    def __init__(self):
        self.motor_deg = 0.0
        self.permille = 0
        self.enabled = False
        self.last_permille = 0
        self._q = []
    def start(self): pass
    def stop(self): pass
    def send(self, can_id, data):
        if can_id != CanSpec.MOTOR_CMD_ID:
            return
        data = bytes(data)
        if data[:4] == CanSpec.CMD_SPEED_HDR:
            self.permille = _decode_value(data[4:8]); self.last_permille = self.permille
        elif data == CanSpec.CMD_ENABLE:
            self.enabled = True
        elif data == CanSpec.CMD_DISABLE:
            self.enabled = False; self.permille = 0
    def step(self, dt):
        rpm = self.permille / 1000.0 * CanSpec.RATED_RPM      # ±80RPM
        self.motor_deg += rpm * 360.0 / 60.0 * dt              # deg
        raw = int(round(self.motor_deg)) & 0xFFFF
        self._q.append((CanSpec.MOTOR_HEARTBEAT_ID,
                        bytes([(raw >> 8) & 0xFF, raw & 0xFF, 0, 0, 0, 0, 0, 0])))
    def recv(self):
        return self._q.pop(0) if self._q else None


def run(target_deg=5.0, steps=1500, dt=0.02):
    sim = KeyaSimCan()
    act = SteeringActuator(sim)
    act.speed_control = True
    act.use_motor_encoder = True
    act.steer_ratio = 17.5
    target = math.radians(target_deg)

    first_cmd = None
    first_nonzero = 0                               # 접근 중 첫 비영 명령(부호 검사용)
    for i in range(steps):
        cmd = act.update(target, 0.0, dt)          # 조향각오차 → cmd_speed 송신
        if i == 0:
            first_cmd = cmd                         # 하트비트 전 → 0 이어야(안전가드)
        if first_nonzero == 0 and cmd != 0:
            first_nonzero = cmd
        sim.step(dt)                                # 모터 적분 + 하트비트
        act.process_can_recv()                      # 하트비트 → 조향각 피드백
    measured = math.degrees(act.get_measured_angle())
    return first_cmd, measured, first_nonzero


if __name__ == "__main__":
    # 1) 안전가드 + 수렴 + 부호(좌, target>0)
    first, meas, perm = run(target_deg=5.0)
    print(f"[좌 +5°] 첫틱cmd(하트비트前)={first}  수렴={meas:.2f}°  last_permille={perm}")
    assert first == 0.0, "안전가드 실패: 하트비트 전 명령이 0이 아님"
    assert abs(meas - 5.0) < 0.5, f"수렴 실패: {meas}"
    assert perm > 0, "부호 실패: target>0(좌) 인데 permille 가 +가 아님"

    # 2) 반대 부호(우, target<0) → permille -
    _, meas2, perm2 = run(target_deg=-5.0)
    print(f"[우 -5°] 수렴={meas2:.2f}°  last_permille={perm2}")
    assert abs(meas2 + 5.0) < 0.5, f"수렴 실패: {meas2}"
    assert perm2 < 0, "부호 실패: target<0(우) 인데 permille 가 -가 아님"

    print("\n  ✓ 무WAS 속도제어 조향 폐루프 검증 통과 "
          "(안전가드/수렴/부호 +permille=좌)")
