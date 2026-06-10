"""
sim_leveling.py
전체 균평 시스템 시뮬레이션 (하드웨어 0)

기존 leveler_core.py 를 그대로 사용. 아무것도 수정 안 함.
MockHydraulics(유압 거동 시뮬)로 실제 트랙터 없이 제어 동작을 검증한다.

검증 항목:
  1. 평탄 목표 수렴 - 블레이드가 목표 높이로 모여 on-grade 유지하는가
  2. 경사 목표 추종 - 트랙터가 전진하며 경사면을 따라가는가
  3. 안전 동작 - RTK fix 끊기면 자동으로 HOLD 하는가

큰형에게 보여줄 데모: python3 sim_leveling.py
"""

from __future__ import annotations
import time
import logging

from leveler_core import (
    LevelerParams, LevelingTuning, LevelerSystem,
    MockHydraulics, GnssFix, Direction,
)

# 로그는 조용히 (시뮬 출력만 보이게)
logging.basicConfig(level=logging.WARNING)


# ─────────────────────────────────────────────────────────────
# 가상 RTK fix 생성기
# ─────────────────────────────────────────────────────────────
# 기준점 (대략 한국 어느 논. 위경도는 임의)
BASE_LAT = 36.5000000
BASE_LON = 127.5000000

# 위경도 1도당 미터 (대략)
M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = 89_000.0   # 위도 36도 부근


def make_fix(east_m: float, north_m: float, alt_m: float,
             quality: int = 4) -> GnssFix:
    """ENU 상대좌표(m)를 위경도+고도 fix로 변환."""
    lat = BASE_LAT + north_m / M_PER_DEG_LAT
    lon = BASE_LON + east_m / M_PER_DEG_LON
    return GnssFix(
        lat=lat, lon=lon, alt=alt_m,
        quality=quality, sats=18, hdop=0.7,
        valid=(quality >= 1),
    )


# ─────────────────────────────────────────────────────────────
# 시뮬레이션 본체
# ─────────────────────────────────────────────────────────────
def run_scenario(title: str, target_mode: str = "flat",
                 slope_pct: float = 0.0, break_rtk_at: float = -1.0):
    print(f"\n{'='*64}")
    print(f"  {title}")
    print(f"{'='*64}")

    params = LevelerParams(
        # MR1157 가정값 (실측 전 기본값)
        antenna_height_above_blade=2.50,
        antenna_to_blade_horizontal=1.20,
        up_speed_cms=8.0,
        down_speed_cms=10.0,
        latency_s=0.30,
        coast_cm=1.5,
        blade_max_up_cm=40.0,
        blade_max_down_cm=-25.0,
    )
    tuning = LevelingTuning(
        on_grade_cm=1.5,    # ±1.5cm 안이면 on-grade
        exit_cm=2.5,
        use_feedforward=True,   # 코스팅 예측 ON
    )

    # 가상 유압 (블레이드 시작 높이 10cm)
    mock = MockHydraulics(params, start_blade_cm=10.0)
    system = LevelerSystem(params, output=mock, tuning=tuning, send_period_s=0.05)

    GROUND_Z = 100.0
    h = params.antenna_height_above_blade

    # ★ ENU 원점을 지면 고도에 명시 고정.
    #   이렇게 하면 블레이드 z 절대값이 "지면 기준 높이"가 된다.
    #   (안 하면 첫 측량점에서 원점이 잡혀 기준이 어긋남)
    system.enu.set_origin(BASE_LAT, BASE_LON, GROUND_Z)

    # ── 측량 단계: 블레이드를 지면에 대고(0cm) 주행하며 표면 측량 ──
    # 블레이드 하단 = 지면(z=100m). 안테나는 그 위 h만큼 → 안테나고도=100+h.
    # origin이 100이므로 측량 블레이드 z ≈ 0 → "지면 = 0cm 목표"가 된다.
    for i in range(11):
        x = i * 2.0   # 0,2,...,20m
        antenna_alt = GROUND_Z + h
        fix = make_fix(east_m=x, north_m=0.0, alt_m=antenna_alt)
        system.add_survey(fix, raw_roll=0.0, raw_pitch=0.0)

    # 목표 평면 설정
    system.fit_plane()
    if target_mode == "flat":
        system.target.set_flat()   # 측량 평균(=지면=0cm)을 평탄 목표로
        print(f"목표: 평탄 (블레이드를 지면 높이 0cm로)")
    elif target_mode == "slope":
        system.target.set_flat()
        system.target.set_slope(slope_east_pct=slope_pct, slope_north_pct=0.0)
        print(f"목표: 동쪽 {slope_pct}% 경사")

    # ── 작업 단계 진입 ──
    system.set_auto(True)
    dt = 0.05            # 50ms 제어주기 (20Hz)
    t = 0.0
    x_pos = 0.0          # 트랙터 동쪽 위치
    speed_ms = 1.2       # 1.2 m/s 전진

    print(f"\n{'t(s)':>5} {'x(m)':>6} {'blade':>7} {'target':>7} "
          f"{'err':>6} {'dir':>5} {'on-grade':>8}  note")

    rtk_quality = 4
    for step in range(200):   # 10초 (200 × 50ms)
        t = step * dt

        # RTK 끊김 시나리오
        note = ""
        if break_rtk_at > 0 and t >= break_rtk_at and t < break_rtk_at + 2.0:
            rtk_quality = 5   # RTK Float (fix 풀림)
            note = "RTK 끊김!"
        else:
            rtk_quality = 4

        # 트랙터 전진
        x_pos += speed_ms * dt

        # MockHydraulics가 관리하는 실제 블레이드 높이(cm) → 안테나 고도(m) 역산
        # 안테나고도 = 지면(100) + 블레이드높이(m) + 안테나↔블레이드(h)
        blade_actual_cm = mock.blade_cm
        antenna_alt = GROUND_Z + blade_actual_cm / 100.0 + h

        # 가상 fix 주입
        fix = make_fix(east_m=x_pos, north_m=0.0, alt_m=antenna_alt, quality=rtk_quality)
        system.on_imu(raw_roll=0.0, raw_pitch=0.0)
        system.on_gnss_fix(fix, now=t)

        # 제어 스텝
        status = system.control_step(now=t)

        # 유압 시뮬 진행
        mock.tick(now=t, dt=dt)

        # 10스텝마다(0.5초) 출력
        if step % 10 == 0:
            err = status.get("error_cm", 0.0)
            tgt = status.get("target_z_cm", 0.0)
            d = status.get("direction", "?")
            og = status.get("on_grade", False)
            d_str = d.name if hasattr(d, "name") else str(d)
            print(f"{t:>5.1f} {x_pos:>6.2f} {blade_actual_cm:>7.2f} {tgt:>7.2f} "
                  f"{err:>+6.2f} {d_str:>5} {'YES' if og else '·':>8}  {note}")

    print(f"\n최종 블레이드 높이: {mock.blade_cm:.2f}cm  (목표 수렴 확인)")


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════════╗
║   FarmMachine 균평 시스템 시뮬레이션 (하드웨어 없이)          ║
║   기존 leveler_core.py 사용 · MockHydraulics 유압 거동 모델   ║
╚══════════════════════════════════════════════════════════════╝
""")

    # 시나리오 1: 평탄 목표 수렴
    run_scenario(
        "시나리오 1 — 평탄 목표 수렴 (블레이드 10cm에서 시작 → 0cm로)",
        target_mode="flat",
    )

    # 시나리오 2: 경사 목표 추종
    run_scenario(
        "시나리오 2 — 동쪽 2% 경사 추종 (전진하며 경사면 따라감)",
        target_mode="slope", slope_pct=2.0,
    )

    # 시나리오 3: 안전 동작 (RTK 끊김)
    run_scenario(
        "시나리오 3 — 안전 정지 (4초 시점 RTK fix 끊김 → HOLD)",
        target_mode="flat", break_rtk_at=4.0,
    )

    print("\n[완료] 세 시나리오 모두 정상 동작하면 제어 로직 검증 OK.")
    print("       실제 하드웨어는 MockHydraulics 자리에 DirectCanOutput 주입.")
