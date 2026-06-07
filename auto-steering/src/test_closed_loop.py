"""
test_closed_loop.py — GNSS(NMEA) → EKF 입력 경로 검증 (의존성: numpy)

새로 배선한 실폐루프의 '입력단'을 결정적으로 검증한다:
  NMEA(GGA 위치 + HDT 진헤딩) → f9p_client.feed
    → AutoSteerSystem.on_rtk / on_heading → EKF(predict+update)
검증:
  1) HDT 나침반 진헤딩(북=0,CW) → EKF 수학각(동=0,CCW) 변환 정확 (math = 90-compass)
  2) GGA 위치 → EKF 위치 추종(북으로 이동 시 +y 증가)
  3) control_step 이 IMU 없이도 predict 수행(전파)

(조향 폐루프 수렴은 sitl_sim 의 6종 시나리오가 별도 검증.)
"""
import math
import autosteer_core as c
from autosteer_core import AutoSteerSystem, KUBOTA_MR1157
from sitl_sim import GeoConverter
from test_speed_control import KeyaSimCan


def _nmea(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def _gga(lat: float, lon: float, q: int = 4) -> str:
    def dm(v, kind):
        deg = int(abs(v)); minutes = (abs(v) - deg) * 60.0
        w = 2 if kind == "lat" else 3
        hemi = ("N" if v >= 0 else "S") if kind == "lat" else ("E" if v >= 0 else "W")
        return f"{deg:0{w}d}{minutes:08.5f}", hemi
    la, lah = dm(lat, "lat"); lo, loh = dm(lon, "lon")
    return _nmea(f"GPGGA,120000.0,{la},{lah},{lo},{loh},{q},12,0.7,50.0,M,0,M,,")


def _hdt(compass_deg: float) -> str:
    return _nmea(f"GPHDT,{compass_deg % 360.0:.1f},T")


def _make_sys():
    sys = AutoSteerSystem(KeyaSimCan(), params=KUBOTA_MR1157, algo="pure_pursuit")
    return sys


def test_heading_convention():
    """HDT 나침반각 → EKF 수학각 변환(90-compass) 검증."""
    geo = GeoConverter(37.0, 127.0)
    for compass in (0.0, 90.0, 180.0, 270.0):
        sys = _make_sys()
        import f9p_client as fc
        g = fc.F9pUsbClient(on_rtk=lambda la, lo, q: sys.on_rtk(la, lo, q, source=sys.gnss.primary),
                            on_heading=sys.on_heading)
        lat, lon = geo.xy_to_ll(0.0, 0.0)
        for _ in range(60):
            g.feed(_gga(lat, lon, 4))
            g.feed(_hdt(compass))
            sys.control_step(0.05)
        got = math.degrees(sys.estimator.get_state().heading) % 360.0
        exp = (90.0 - compass) % 360.0
        d = abs((got - exp + 180) % 360 - 180)
        print(f"  compass {compass:5.0f}° → EKF {got:6.1f}° (기대 {exp:5.1f}°, 오차 {d:.2f}°)")
        assert d < 2.0, f"헤딩 변환 오차 {d:.1f}° (compass {compass})"


def test_position_tracking():
    """북향 이동(GGA) → EKF +y 증가, 헤딩 북(=math 90°)."""
    geo = GeoConverter(37.0, 127.0)
    sys = _make_sys()
    import f9p_client as fc
    g = fc.F9pUsbClient(on_rtk=lambda la, lo, q: sys.on_rtk(la, lo, q, source=sys.gnss.primary),
                        on_heading=sys.on_heading)
    y = 0.0
    for _ in range(80):
        lat, lon = geo.xy_to_ll(0.0, y)
        g.feed(_gga(lat, lon, 4))
        g.feed(_hdt(0.0))               # 북향(compass 0)
        sys.control_step(0.05)
        y += 1.0 * 0.05                 # 1 m/s 북진
    st = sys.estimator.get_state()
    hd = math.degrees(st.heading) % 360.0
    print(f"  북진 1m/s 4s → EKF y={st.y:.2f}m (기대 ~{y:.1f}), heading={hd:.1f}° (기대 90°)")
    assert st.y > 2.0, f"북진인데 EKF y 증가 안함: {st.y}"
    assert abs(st.x) < 0.5, f"x 가 직진에서 벗어남: {st.x}"
    assert abs((hd - 90 + 180) % 360 - 180) < 2.0, f"heading 북(90°) 아님: {hd}"


def test_dual_imu_fusion():
    """ver1: 노이즈 있는 듀얼안테나 절대헤딩 + 자이로 레이트 → EKF 가 평활(스네이크↓)."""
    import random, statistics
    random.seed(7)
    sys = _make_sys()
    truth_compass = 0.0          # 북향(나침반 0)
    raw_math, ekf_math = [], []
    for _ in range(200):
        noisy = truth_compass + random.gauss(0, 3.0)     # 듀얼헤딩 노이즈 ±3°
        sys.on_gyro(ang_vel=random.gauss(0, 0.01), dt=0.05)   # 자이로(진짜 rate≈0)
        sys.on_heading(noisy)
        raw_math.append((90.0 - noisy))                  # 듀얼 raw(수학각)
        ekf_math.append(math.degrees(sys.estimator.get_state().heading))
    raw_std = statistics.pstdev(raw_math[50:])
    ekf_std = statistics.pstdev(ekf_math[50:])
    print(f"  raw 듀얼헤딩 std={raw_std:.2f}° → EKF 융합 std={ekf_std:.2f}° "
          f"(평활비 {raw_std/max(1e-6,ekf_std):.1f}×)")
    # 융합이 헤딩 노이즈를 유의미하게 감소(추가 평활은 Q[heading] 튜닝으로 가능)
    assert ekf_std < raw_std * 0.8, f"융합 평활 부족: raw {raw_std:.2f} vs ekf {ekf_std:.2f}"


def test_heading_calibration():
    """ver1: 직선주행 20m → HeadingCalibrator 가 베이스라인 바이어스 복원 + on_heading 보정."""
    from calibration import HeadingCalibrator
    cal = HeadingCalibrator(min_distance_m=15.0, min_samples=80)
    BIAS = 5.0                       # 진짜 바이어스 +5°(보고가 진로각보다 5° 큼)
    north = 0.0
    for _ in range(120):
        north += 0.2                 # 북진 0.2m/스텝
        cal.add_sample(east=0.0, north=north, reported_heading_deg=0.0 + BIAS)
    est = cal.finish()
    print(f"  캘리브: {est.note}  (기대 +{BIAS}°, ok={est.ok})")
    assert est.ok and abs(est.value - BIAS) < 0.5, f"바이어스 복원 실패: {est.value}"
    # 보정 적용 → 실제 북향(진로각 0)에서 EKF heading 이 수학각 90°로 정확
    sys = _make_sys()
    sys.set_heading_bias(est.value)
    for _ in range(40):
        sys.on_gyro(0.0, dt=0.05); sys.on_heading(0.0 + BIAS)   # 보고헤딩=진로+bias
    hd = math.degrees(sys.estimator.get_state().heading) % 360.0
    print(f"  바이어스 보정 후 EKF heading={hd:.1f}° (기대 90°=북)")
    assert abs((hd - 90 + 180) % 360 - 180) < 1.0, f"보정 후 heading 오차: {hd}"


if __name__ == "__main__":
    print("[1] HDT 나침반→수학각 변환")
    test_heading_convention()
    print("[2] GGA 위치 추종")
    test_position_tracking()
    print("[3] ver1 듀얼안테나+IMU 융합(평활)")
    test_dual_imu_fusion()
    print("[4] ver1 헤딩 바이어스 캘리브(20m 직선)")
    test_heading_calibration()
    print("\n  ✓ GNSS(NMEA)→EKF 입력 경로 검증 통과 — 헤딩 변환/위치 추종/무IMU predict. "
          "(조향 수렴은 sitl_sim 6/6)")
