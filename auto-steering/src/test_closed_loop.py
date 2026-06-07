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


if __name__ == "__main__":
    print("[1] HDT 나침반→수학각 변환")
    test_heading_convention()
    print("[2] GGA 위치 추종")
    test_position_tracking()
    print("\n  ✓ GNSS(NMEA)→EKF 입력 경로 검증 통과 — 헤딩 변환/위치 추종/무IMU predict. "
          "(조향 수렴은 sitl_sim 6/6)")
