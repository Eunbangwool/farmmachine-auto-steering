"""
proportional_valve.py — 비례밸브 직결 균평 출력단 (방식 C) + PID/PWM 제어
==========================================================================
leveler_core.py 의 LevelerOutput 추상화에 얹는 **add-on** (코어 미수정).
direct_can_output.py 와 동일한 add-on 패턴.

기존 leveler_core 의 LevelingController 는 레이저커넥터(EH 밸브 UP/DOWN/HOLD,
트랙터가 유압을 알아서 제어)용 히스테리시스 제어다. 이 파일은 **비례밸브를
앱이 직접 PWM 으로 구동**하는 경로(LevelerOutput 방식 C — 코어에 "미구현"으로
표시돼 있던 TODO)를 채운다.

★ 출처(기능적 사실만 이식, clean-room): `CHCNAV_PARAM_PROFILE.md §8`
   — CHCNAV NX510 `libGNSSBladeControl` .so 정적분석으로 복원한 로직.
   복원된 사실:
     · PWM 듀티 범위 10%(최소작동)~96%(상한)
     · 고도오차 데드존 다단계 (0.0001~0.07 m, mm급)
     · 차속<최소속도 → 무조건 HOLD / 데드존 내(up_flag·down_flag=0) → HOLD
     · 비례밸브 캘리브 4파라미터(중립전압/최대전압/비례계수/오프셋),
       파일 `/sdcard/ControllerX/IC100Paras.txt` (`%.3lf` 4값)
     · 신호필터 = 칼만 + 이동평균,  제어기 = PID
     · 출력 = Send_pwm → CAN(send_can_message_C/CAN2) 또는 GPIO(sendDirectOutCtl)
   소스 복붙 아님 — 위 수치/구조를 보고 자체 구현. PID 게인 등 미기재 값은 ★추정(현장튜닝).

의존성: 표준 라이브러리 + leveler_core. numpy 불필요.
    cd rtk-leveling/src && python proportional_valve.py
"""
from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from leveler_core import (LevelerOutput, Direction, CanBus, LevelerCanProtocol)

log = logging.getLogger("leveler.propvalve")


# ═══════════════════════════════════════════════════════════════
#  CHCNAV 복원 프로파일 (§8) — 모든 수치에 출처/단위 주석
# ═══════════════════════════════════════════════════════════════
@dataclass
class ChcnavBladeProfile:
    """CHCNAV_PARAM_PROFILE.md §8 복원값 기반 비례밸브 제어 프로파일."""
    # PWM 듀티 한계 (§8: 10% 최소작동 ~ 96% 상한)
    pwm_min_duty: float = 0.10
    pwm_max_duty: float = 0.96
    # 방향별 듀티 스케일 (§8: PWM_Up / PWM_Down 별개. 보통 자중으로 하강이 빠름 → DOWN 약하게)
    up_duty_scale: float = 1.0
    down_duty_scale: float = 0.85    # ★추정(자중 하강) — 현장 캘리브로 조정

    # 다단계 고도오차 데드존 (§8: 0.0001~0.07 m). 단위 m.
    #   on_grade_m  : |err| < 이 값 → HOLD ('온그레이드', up/down_flag=0). mm급.
    #   fine_m      : on_grade~fine 구간은 최소~비례 듀티로 미세제어
    #   coarse_m    : 이 이상 → 최대 듀티(거친 구간). 0.07 m 상한 부근.
    on_grade_m: float = 0.005        # 5 mm (★ 0.0001 m 까지 가능하나 RTK 잡음 고려 기본 5mm)
    fine_m: float = 0.02             # 20 mm
    coarse_m: float = 0.07           # 70 mm (§8 상한)

    # 차속 게이트 (§8: 차속<최소 → 무조건 HOLD). m/s.
    min_speed_mps: float = 0.15      # ★추정 — 현장확인(정지/저속 시 오제어 방지)

    # PID 게인 (§8: 제어기=PID. 게인값은 .so 미기재 → ★추정, 현장튜닝)
    #   effort 정규화: 오차 ≈ coarse_m(0.07 m) 에서 |effort|≈1 포화하도록 per-meter 스케일.
    kp: float = 14.0                 # 1/m  (≈ 1/coarse_m → coarse 오차에서 포화)
    ki: float = 2.0                  # 1/(m·s)
    kd: float = 3.0                  # s/m  (오버슈트 댐핑)
    i_limit: float = 0.5             # 적분 와인드업 제한 (effort)

    # 신호필터 (§8: 칼만 + 이동평균)
    kalman_q: float = 1e-4           # 프로세스 분산 (m²) — 작을수록 신뢰 모델
    kalman_r: float = 4e-4           # 측정 분산 (m²) ≈ (2cm)² RTK 잡음 ★현장
    ma_window: int = 5               # 이동평균 창

    invert: bool = False             # 블레이드 높음=DOWN 기본. 반대 기구면 True


# ═══════════════════════════════════════════════════════════════
#  신호 필터 — 칼만(1D) + 이동평균 (§8)
# ═══════════════════════════════════════════════════════════════
class SignalFilter:
    """블레이드 높이/오차 신호용 1D 칼만 + 이동평균. RTK·IMU 잡음 평활."""
    def __init__(self, q: float, r: float, window: int):
        self.q = q
        self.r = r
        self._x: Optional[float] = None   # 칼만 추정
        self._p = 1.0                     # 추정 분산
        self._ma: deque = deque(maxlen=max(1, window))

    def reset(self):
        self._x = None
        self._p = 1.0
        self._ma.clear()

    def update(self, z: float) -> float:
        # 1D 칼만 (정적 모델: x_k = x_{k-1})
        if self._x is None:
            self._x = z
        else:
            self._p += self.q
            k = self._p / (self._p + self.r)      # 칼만 이득
            self._x += k * (z - self._x)
            self._p *= (1 - k)
        # 이동평균(칼만 출력 위에 한 번 더 — §8 칼만+MA 직렬)
        self._ma.append(self._x)
        return sum(self._ma) / len(self._ma)


# ═══════════════════════════════════════════════════════════════
#  PID
# ═══════════════════════════════════════════════════════════════
class Pid:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self._i = 0.0
        self._prev: Optional[float] = None

    def reset(self):
        self._i = 0.0
        self._prev = None

    def step(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        self._i += error * dt
        self._i = max(-self.i_limit / max(self.ki, 1e-9),
                      min(self.i_limit / max(self.ki, 1e-9), self._i))
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        return self.kp * error + self.ki * self._i + self.kd * d


# ═══════════════════════════════════════════════════════════════
#  비례밸브 캘리브레이션 (§8: IC100Paras.txt, 4파라미터)
# ═══════════════════════════════════════════════════════════════
@dataclass
class IC100Calibration:
    """
    비례밸브 4파라미터 (§8). 듀티(0~1) → 밸브 구동전압(V) 매핑.
    파일포맷: `/sdcard/ControllerX/IC100Paras.txt`, '%.3lf' 4값 (한 줄 또는 줄바꿈).
    """
    neutral_v: float = 0.0           # 중립전압 (밸브 정지)
    max_v: float = 10.0              # 최대전압 (full open)
    prop_coeff: float = 1.0          # 비례계수 (듀티 선형성 보정)
    offset_v: float = 0.0            # 오프셋(데드밴드 보상 시동전압)

    def duty_to_voltage(self, duty: float) -> float:
        """듀티(0~1) → 구동전압. 0이면 중립. duty>0 이면 offset 시동전압부터."""
        if duty <= 0.0:
            return self.neutral_v
        span = (self.max_v - self.neutral_v)
        v = self.neutral_v + self.offset_v + self.prop_coeff * duty * (span - self.offset_v)
        return max(self.neutral_v, min(self.max_v, v))

    @classmethod
    def load(cls, path: str) -> "IC100Calibration":
        """IC100Paras.txt 로드. 실패 시 기본값(사유 로그) — 조용히 먹지 않음."""
        try:
            with open(path) as f:
                nums = [float(t) for t in f.read().replace("\n", " ").split()]
            if len(nums) < 4:
                raise ValueError(f"4값 필요, {len(nums)}개")
            return cls(neutral_v=nums[0], max_v=nums[1],
                       prop_coeff=nums[2], offset_v=nums[3])
        except Exception as e:
            log.warning(f"IC100Paras 로드 실패({path}): {e} → 기본값")
            return cls()

    def save(self, path: str):
        with open(path, "w") as f:
            f.write("%.3lf %.3lf %.3lf %.3lf\n" % (
                self.neutral_v, self.max_v, self.prop_coeff, self.offset_v))


# ═══════════════════════════════════════════════════════════════
#  비례밸브 제어기 (PID → 다단계 데드존 → PWM 듀티)  ★방식 C 핵심
# ═══════════════════════════════════════════════════════════════
@dataclass
class PropValveCommand:
    direction: Direction
    duty: float                      # 0 또는 [pwm_min_duty, pwm_max_duty]
    voltage: float                   # 캘리브 적용 구동전압 (참고/GPIO 경로)
    on_grade: bool
    reason: str                      # HOLD 사유("" = 구동)


class ProportionalValveController:
    """
    블레이드 z → 목표 z 오차를 PID + 다단계 데드존으로 PWM 듀티 명령으로 변환.
    CHCNAV §8 로직 재현: 차속<최소→HOLD, 데드존내→HOLD, 그 외 UP/DOWN PWM.
    """
    def __init__(self, profile: ChcnavBladeProfile = None,
                 calib: IC100Calibration = None):
        self.p = profile or ChcnavBladeProfile()
        self.calib = calib or IC100Calibration()
        self.filter = SignalFilter(self.p.kalman_q, self.p.kalman_r, self.p.ma_window)
        self.pid = Pid(self.p.kp, self.p.ki, self.p.kd, self.p.i_limit)

    def reset(self):
        self.filter.reset()
        self.pid.reset()

    def _duty_envelope(self, abs_err_m: float) -> float:
        """다단계 데드존(§8) → 듀티 상한 스케일 [0,1]. on_grade 미만은 0(HOLD)."""
        p = self.p
        if abs_err_m < p.on_grade_m:
            return 0.0
        if abs_err_m >= p.coarse_m:
            return 1.0                     # 거친 구간 → 최대듀티 허용
        # on_grade~coarse 사이 선형 (fine 지점에서 중간) — mm급 미세제어
        return (abs_err_m - p.on_grade_m) / max(1e-6, (p.coarse_m - p.on_grade_m))

    def compute(self, blade_z_m: float, target_z_m: float,
                speed_mps: float, dt: float) -> PropValveCommand:
        p = self.p
        # 1) 신호 필터(칼만+MA) — 블레이드 높이 평활
        z = self.filter.update(blade_z_m)
        error = z - target_z_m             # +면 블레이드 높음 → DOWN 필요

        # 2) 차속 게이트 (§8: 차속<최소 → HOLD)
        if abs(speed_mps) < p.min_speed_mps:
            self.pid.reset()               # 정지 중 적분 누적 방지
            return PropValveCommand(Direction.NEUTRAL, 0.0, self.calib.neutral_v,
                                    on_grade=False, reason="below_min_speed")

        # 3) 데드존 (§8: up_flag·down_flag 둘 다 0 → HOLD = 온그레이드)
        abs_err = abs(error)
        env = self._duty_envelope(abs_err)
        if env <= 0.0:
            return PropValveCommand(Direction.NEUTRAL, 0.0, self.calib.neutral_v,
                                    on_grade=True, reason="on_grade")

        # 4) PID effort → 듀티(envelope 로 다단계 상한, [min,max] 클램프)
        effort = self.pid.step(error, dt)          # 부호=방향, 크기=세기
        raw_down = (effort > 0)                    # 높음(+error)→DOWN
        if p.invert:
            raw_down = not raw_down
        direction = Direction.DOWN if raw_down else Direction.UP
        dir_scale = p.down_duty_scale if direction == Direction.DOWN else p.up_duty_scale

        # 다단계 데드존(env, 상한)과 PID 세기 중 작은 값 → 듀티 크기
        mag = min(env, min(1.0, abs(effort))) * dir_scale
        # 최소작동 듀티 보장(§8: 10% 미만은 밸브가 안 움직임)
        duty = p.pwm_min_duty + mag * (p.pwm_max_duty - p.pwm_min_duty)
        duty = max(p.pwm_min_duty, min(p.pwm_max_duty, duty))
        return PropValveCommand(direction, duty, self.calib.duty_to_voltage(duty),
                                on_grade=False, reason="")


# ═══════════════════════════════════════════════════════════════
#  방식 C 출력단 — 비례밸브 직결 (PWM 듀티 → CAN)
# ═══════════════════════════════════════════════════════════════
# 듀티(0~1)를 LevelerCanProtocol CMD 프레임 Byte2(PulseWidth)에 0~254 로 인코딩
# (255=연속 예약). 컨트롤러(STM32/비례밸브 드라이버)가 이 값을 PWM 듀티로 해석.
# §8 출력경로: Send_pwm → send_can_message_C/CAN2. GPIO 직접출력(sendDirectOutCtl)
# 경로는 voltage(캘리브 적용)를 별도 아날로그/GPIO 드라이버로 전달(여기선 CAN 우선).
class ProportionalValveOutput(LevelerOutput):
    """방식 C — 비례밸브 직결. send_command 의 pulse_ms 를 듀티(0~1)로 재해석."""
    DUTY_CONTINUOUS = 255

    def __init__(self, bus: CanBus):
        self.bus = bus
        self.last_duty = 0.0
        self.last_voltage = 0.0

    def start(self): self.bus.start()
    def stop(self):  self.bus.stop()

    @staticmethod
    def duty_to_byte(duty: float) -> int:
        return max(0, min(254, int(round(max(0.0, min(1.0, duty)) * 254))))

    def send_command(self, mode, direction, pulse_ms, heartbeat, now=None):
        """pulse_ms 인자를 듀티(0~1)로 사용(방식 C 규약). 0=HOLD."""
        duty = max(0.0, min(1.0, float(pulse_ms)))
        self.last_duty = duty
        byte = self.duty_to_byte(duty)
        can_id, data = LevelerCanProtocol.encode_cmd(mode, direction, byte, heartbeat)
        try:
            self.bus.send(can_id, data, now=now)
        except TypeError:
            self.bus.send(can_id, data)

    def send_valve(self, cmd: PropValveCommand, mode: int, heartbeat: int,
                   now: float = None):
        """PropValveCommand 직접 송신(듀티+전압 보관)."""
        self.last_voltage = cmd.voltage
        self.send_command(mode, cmd.direction, cmd.duty, heartbeat, now=now)


# ═══════════════════════════════════════════════════════════════
#  테스트용 비례밸브 유압 시뮬 (블레이드 속도 ∝ 듀티)
# ═══════════════════════════════════════════════════════════════
class MockProportionalHydraulics(CanBus):
    """듀티에 비례하는 블레이드 속도 + dead time. 실제로는 STM32+비례밸브."""
    def __init__(self, max_speed_cms: float = 12.0, latency_s: float = 0.2,
                 start_blade_cm: float = 10.0):
        self.max_speed_cms = max_speed_cms
        self.latency_s = latency_s
        self.blade_cm = start_blade_cm
        self._dir = Direction.NEUTRAL
        self._duty = 0.0
        self._cmd_t = -1e9
        self._was_active = False
        self._last = time.time()

    def start(self): pass
    def stop(self): pass

    def send(self, can_id: int, data: bytes, now: float = None):
        if can_id != LevelerCanProtocol.CMD_ID:
            return
        _now = now if now is not None else time.time()
        d = Direction(data[1])
        duty = data[2] / 254.0
        active = (d != Direction.NEUTRAL and duty > 0.0)
        # dead time 은 정지→구동 전환 때만 시작(구동 지속 중 재명령은 리셋 안 함)
        if active and not self._was_active:
            self._cmd_t = _now
        self._was_active = active
        self._dir = d
        self._duty = duty

    def tick(self, now: float = None, dt: float = None):
        _now = now if now is not None else time.time()
        dt = (_now - self._last) if dt is None else dt
        self._last = _now
        moving = (self._dir != Direction.NEUTRAL and self._duty > 0
                  and (_now - self._cmd_t) >= self.latency_s)
        if moving:
            v = self.max_speed_cms * self._duty
            self.blade_cm += (v if self._dir == Direction.UP else -v) * dt


# ═══════════════════════════════════════════════════════════════
#  자가검증 (HW 불필요)
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import os, tempfile
    print("=" * 70)
    print("proportional_valve — CHCNAV §8 비례밸브 PID/PWM 이식 자가검증")
    print("=" * 70)

    prof = ChcnavBladeProfile()
    ctl = ProportionalValveController(prof)

    # ── 1. 신호필터: 잡음 분산 감소 ───────────────────────────
    print("\n▶ 1. 칼만+이동평균 신호필터")
    f = SignalFilter(prof.kalman_q, prof.kalman_r, prof.ma_window)
    truth = 1.0
    noisy = [truth + (0.02 if i % 2 else -0.02) for i in range(40)]
    out = [f.update(z) for z in noisy]
    var_in = sum((z - truth) ** 2 for z in noisy) / len(noisy)
    var_out = sum((z - truth) ** 2 for z in out[10:]) / len(out[10:])
    assert var_out < var_in * 0.25, (var_in, var_out)
    print(f"  ✓ 입력분산 {var_in:.5f} → 출력분산 {var_out:.5f} (≥75% 감소)")

    # ── 2. 데드존 HOLD (§8 온그레이드) ───────────────────────
    print("\n▶ 2. 데드존(온그레이드) HOLD")
    c = ctl.compute(blade_z_m=1.000, target_z_m=1.000 + prof.on_grade_m * 0.5,
                    speed_mps=1.0, dt=0.05)
    assert c.direction == Direction.NEUTRAL and c.duty == 0.0 and c.on_grade
    print(f"  ✓ |오차|<{prof.on_grade_m*1000:.1f}mm → HOLD(on_grade), 듀티 0")

    # ── 3. 차속 게이트 HOLD (§8) ──────────────────────────────
    print("\n▶ 3. 차속<최소 → 무조건 HOLD")
    c = ctl.compute(1.0, 1.10, speed_mps=0.05, dt=0.05)   # 큰 오차지만 저속
    assert c.direction == Direction.NEUTRAL and c.reason == "below_min_speed"
    print(f"  ✓ 속도 0.05<{prof.min_speed_mps} → HOLD (큰 오차 무시)")

    # ── 4. 듀티 한계 [10%,96%] + 방향 ─────────────────────────
    print("\n▶ 4. PWM 듀티 한계/방향")
    ctl.reset()
    c_hi = None
    for _ in range(10):                       # 큰 오차(블레이드 높음) → DOWN, 듀티 포화
        c_hi = ctl.compute(2.0, 1.0, speed_mps=1.0, dt=0.05)
    assert c_hi.direction == Direction.DOWN, c_hi.direction
    assert prof.pwm_min_duty <= c_hi.duty <= prof.pwm_max_duty
    assert c_hi.duty > 0.7, c_hi.duty        # 포화(자중 하강 스케일 0.85 반영)
    ctl.reset()
    c_up = None
    for _ in range(10):                       # 큰 오차(블레이드 낮음) → UP, 96% 상한 도달
        c_up = ctl.compute(1.0, 2.0, speed_mps=1.0, dt=0.05)
    assert c_up.direction == Direction.UP
    assert abs(c_up.duty - prof.pwm_max_duty) < 1e-6, c_up.duty
    print(f"  ✓ 포화 DOWN 듀티={c_hi.duty:.2f}(×0.85) / UP 듀티={c_up.duty:.2f}(=96%상한)")

    # ── 5. IC100 캘리브 4파라미터 저장/로드/매핑 ──────────────
    print("\n▶ 5. IC100Paras.txt 4파라미터")
    cal = IC100Calibration(neutral_v=0.5, max_v=9.0, prop_coeff=1.0, offset_v=0.8)
    p_tmp = os.path.join(tempfile.gettempdir(), "IC100Paras.txt")
    cal.save(p_tmp)
    cal2 = IC100Calibration.load(p_tmp)
    assert abs(cal2.max_v - 9.0) < 1e-3 and abs(cal2.offset_v - 0.8) < 1e-3
    assert cal2.duty_to_voltage(0.0) == 0.5                    # 중립
    v10, v96 = cal2.duty_to_voltage(0.10), cal2.duty_to_voltage(0.96)
    assert 0.5 < v10 < v96 <= 9.0                              # 단조증가, 상한
    print(f"  ✓ 저장/로드 OK | 듀티10%→{v10:.2f}V, 96%→{v96:.2f}V (중립0.5~최대9.0)")

    # ── 6. 폐루프 수렴 (비례밸브 유압 시뮬) ───────────────────
    print("\n▶ 6. 폐루프 수렴 (MockProportionalHydraulics)")
    bus = MockProportionalHydraulics(max_speed_cms=12.0, latency_s=0.2,
                                     start_blade_cm=30.0)
    out_c = ProportionalValveOutput(bus)
    ctl.reset()
    target_m = 0.0
    DT, hb = 0.05, 0
    t = time.time()
    for i in range(600):                      # 최대 30초
        t += DT
        bus.tick(now=t, dt=DT)
        blade_m = bus.blade_cm / 100.0
        cmd = ctl.compute(blade_m, target_m, speed_mps=1.0, dt=DT)
        hb = (hb + 1) & 0xFF
        out_c.send_valve(cmd, LevelerCanProtocol.MODE_AUTO, hb, now=t)
        if i > 200 and abs(bus.blade_cm) < prof.on_grade_m * 100:
            break
    assert abs(bus.blade_cm) < prof.on_grade_m * 100 + 0.5, f"미수렴 {bus.blade_cm:.2f}cm"
    print(f"  ✓ 30cm→0 수렴: 최종 블레이드 {bus.blade_cm:.2f}cm (온그레이드 ±{prof.on_grade_m*100:.1f}cm)")

    # ── 7. CAN 인코딩(듀티→Byte2) + CRC ───────────────────────
    print("\n▶ 7. 방식C CAN 인코딩")
    byte = ProportionalValveOutput.duty_to_byte(0.96)
    cid, data = LevelerCanProtocol.encode_cmd(
        LevelerCanProtocol.MODE_AUTO, Direction.DOWN, byte, 7)
    assert cid == LevelerCanProtocol.CMD_ID
    assert data[7] == LevelerCanProtocol.crc8(bytes(data[:7]))
    assert data[1] == Direction.DOWN.value and data[2] == byte
    print(f"  ✓ 듀티0.96→Byte2={byte}, dir=DOWN, CRC8 검증")

    print("\n" + "=" * 70)
    print("proportional_valve (CHCNAV §8 이식) 자가검증 7/7 통과.")
    print("  실배포: ProportionalValveOutput(ApolloCanBus) + IC100Paras.txt 현장 캘리브,")
    print("  PID 게인/min_speed/kalman_r 는 ★현장튜닝(.so 미기재값).")
