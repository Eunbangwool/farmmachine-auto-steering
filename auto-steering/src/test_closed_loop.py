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


def _relposned(heading_deg, acc_deg, baseline_m=1.0, rel_d_m=0.0,
               fix_ok=True, valid=True, heading_valid=True, carr=2,
               version=1):
    """테스트용 UBX-NAV-RELPOSNED 프레임 합성(체크섬 포함)."""
    import struct, f9p_client as fc
    n = 64 if version == 1 else 40
    payload = bytearray(n)
    payload[0] = version
    if version == 1:
        struct.pack_into("<i", payload, 16, int(round(rel_d_m * 100)))      # relPosD cm
        struct.pack_into("<i", payload, 20, int(round(baseline_m * 100)))   # relPosLength cm
        struct.pack_into("<i", payload, 24, int(round(heading_deg * 1e5)))  # relPosHeading
        struct.pack_into("<I", payload, 52, int(round(acc_deg * 1e5)))      # accHeading
        flags = 0
        if fix_ok: flags |= 0x01
        if valid: flags |= 0x04
        flags |= (carr & 0x03) << 3
        if heading_valid: flags |= 0x100
        struct.pack_into("<I", payload, 60, flags)
    body = bytes([0x01, 0x3C, n & 0xFF, (n >> 8) & 0xFF]) + bytes(payload)
    cka, ckb = fc._ubx_checksum(body)
    return b"\xb5\x62" + body + bytes([cka, ckb])


def test_parse_relposned():
    """무빙베이스 RELPOSNED 파싱 + 체크섬/버전 가드."""
    import f9p_client as fc
    f = _relposned(123.456, 0.25, baseline_m=1.5, rel_d_m=-0.26, carr=2)
    m = fc.parse_relposned(f)
    assert m is not None, "정상 프레임 파싱 실패"
    assert abs(m["heading_deg"] - 123.456) < 1e-3, m["heading_deg"]
    assert abs(m["acc_deg"] - 0.25) < 1e-3, m["acc_deg"]
    assert abs(m["baseline_m"] - 1.5) < 1e-3, m["baseline_m"]
    assert m["fix_ok"] and m["valid"] and m["heading_valid"] and m["carr_soln"] == 2
    print(f"  RELPOSNED 파싱 OK: heading={m['heading_deg']:.3f}° acc={m['acc_deg']:.3f}° "
          f"baseline={m['baseline_m']:.2f}m carr={m['carr_soln']}")
    bad = bytearray(_relposned(10.0, 0.2)); bad[-1] ^= 0xFF      # 체크섬 손상
    assert fc.parse_relposned(bytes(bad)) is None, "체크섬 오류인데 파싱됨"
    assert fc.parse_relposned(_relposned(10.0, 0.2, version=0)) is None, "v0(40B) 거부 안됨"
    print("  손상 체크섬 / v0 거부 OK")


def test_adaptive_heading_R():
    """accHeading(에폭별 정확도)로 R 조절 — 저-σ 측정이 더 크게 반영."""
    sys = _make_sys()
    e = sys.estimator
    def one(sigma_deg):
        e.x[:] = 0.0
        e.P = e.np.eye(5) * 0.1; e.P[2, 2] = 0.001
        e.update_heading_adaptive(math.radians(45.0), math.radians(sigma_deg))
        return e.x[2]
    low = one(0.2); high = one(2.0)
    print(f"  acc=0.2° → {math.degrees(low):.1f}° vs acc=2.0° → {math.degrees(high):.1f}° "
          f"(목표 45°)")
    assert low > high, "저-σ 가 더 크게 반영되지 않음"
    assert abs(math.degrees(low) - 45.0) < 6.0, "저-σ 측정이 목표에 못 미침"


def test_heading_gating():
    """fix 게이팅: fixed/유효만 수용, float·invalid·과대σ 는 헤딩 미갱신."""
    sys = _make_sys()
    e = sys.estimator
    def feed(**kw):
        e.x[2] = 0.0; e.P = e.np.eye(5) * 0.1
        m = dict(heading_deg=45.0, acc_deg=0.2, baseline_m=1.0, rel_d_m=0.0,
                 fix_ok=True, valid=True, heading_valid=True, carr_soln=2)
        m.update(kw); sys.on_heading_meas(m)
        return abs(e.x[2])
    assert feed() > 0.1, "fixed 유효 헤딩이 수용 안됨"
    assert feed(carr_soln=1) < 1e-9, "float 헤딩이 거부 안됨"
    assert feed(carr_soln=0) < 1e-9, "비RTK 헤딩이 거부 안됨"
    assert feed(valid=False) < 1e-9, "relPosValid=False 거부 안됨"
    assert feed(heading_valid=False) < 1e-9, "headingValid=False 거부 안됨"
    assert feed(acc_deg=2.0) < 1e-9, "과대 정확도(2°) 거부 안됨"
    print("  fixed 수용 / float·invalid·과대σ 거부 OK")


def test_tilt_compensation():
    """방법 4: RELPOSNED 베이스라인 down 성분 → 차체 틸트(roll) → 경사보정에 공급."""
    sys = _make_sys()
    # base=좌/rover=우: rover(우) 안테나가 0.1736m 하강 → relPosD=+0.1736, baseline=1.0
    #   → roll = +asin(0.1736) = +10° (우측 하강).
    sys.on_heading_meas(dict(heading_deg=90.0, acc_deg=0.2, baseline_m=1.0,
                             rel_d_m=0.1736, fix_ok=True, valid=True,
                             heading_valid=True, carr_soln=2))
    roll = math.degrees(sys.estimator._current_roll)
    print(f"  베이스라인 틸트(우측하강) → roll={roll:.2f}° (기대 +10°)")
    assert abs(roll - 10.0) < 0.5, f"틸트 유도 오차: {roll}"
    # 경사보정 배선 확인: roll 설정 시 update_rtk 가 횡방향 보정을 반영
    from calibration import RollPitchEstimator
    rp = RollPitchEstimator()
    g = 9.80665
    for _ in range(200):                       # 정적: 10° 기운 중력벡터
        r = rp.update(ay=g*math.sin(math.radians(10)), az=g*math.cos(math.radians(10)),
                      lin_acc=0.0, yaw_rate=0.0, dt=0.02)
    print(f"  RollPitchEstimator 정적 roll={math.degrees(r):.2f}° (기대 10°)")
    assert abs(math.degrees(r) - 10.0) < 1.0, f"가속도 roll 추정 오차: {math.degrees(r)}"
    # 회전 중(높은 yaw rate)에는 가속도 보정 무시 → 자이로만
    r2 = rp.update(ay=g, az=0.0, lin_acc=0.0, yaw_rate=1.0, dt=0.02)
    assert abs(math.degrees(r2) - 10.0) < 1.5, "원심가속 오염 배제 실패"
    print("  원심가속 구간 가속도-보정 배제 OK")


def test_dual_mount_offset():
    """base=좌/rover=우: 베이스라인 벡터=우현 → relPosHeading=차체+90° → 90° 차감해 차체헤딩 복원."""
    sys = _make_sys()
    # 차체가 정북(나침반 0)일 때 베이스라인(좌→우)은 정동 → relPosHeading=90°.
    for _ in range(30):
        sys.estimator.predict(0.05)
        sys.on_heading_meas(dict(heading_deg=90.0, acc_deg=0.2, baseline_m=1.0,
                                 rel_d_m=0.0, fix_ok=True, valid=True,
                                 heading_valid=True, carr_soln=2))
    hd = math.degrees(sys.estimator.get_state().heading) % 360.0
    print(f"  baseline heading 90°(우현) → 차체 EKF {hd:.1f}° (기대 90°=북, 수학각)")
    assert abs((hd - 90 + 180) % 360 - 180) < 1.0, f"마운트 오프셋 복원 실패: {hd}"


def test_mount_diagnostic():
    """현장 진단: 직선주행 중 relPosHeading=course+90 → base=좌 추천. 기울임 → roll 부호."""
    from calibration import DualMountDiagnostic
    d = DualMountDiagnostic(min_distance_m=15.0, min_samples=60)
    north = 0.0
    for _ in range(120):                       # 북진(course≈0), 베이스라인=차체+90 → relPosHeading≈90
        north += 0.25
        d.add_sample(east=0.0, north=north, relpos_heading_deg=90.0,
                     rel_d_m=0.0, baseline_m=0.6, acc_deg=0.4)
    # 마지막에 우측으로 기울임(우 안테나 하강 → relPosD>0)
    d.add_sample(east=0.0, north=north + 0.25, relpos_heading_deg=90.0,
                 rel_d_m=0.08, baseline_m=0.6, acc_deg=0.4)
    r = d.report()
    print(f"  offset={r['offset_deg']:+.1f}° → base={r['base_antenna']} "
          f"(추천 offset={r['rec_baseline_offset_deg']:+.0f}°, roll_sign={r['rec_dual_roll_sign']}), "
          f"baseline={r['baseline_m']}m")
    assert r["ready"], "샘플 부족"
    assert abs(r["rec_baseline_offset_deg"] - 90.0) < 1e-6, "base=좌(+90°) 추천 실패"
    assert "좌" in r["base_antenna"], r["base_antenna"]
    assert r["rec_dual_roll_sign"] == 1.0, "우측하강 relPosD>0 → roll +1 추천 실패"


def test_cog_aiding():
    """방법 5: 진로각(COG) 보조 — 잡음 수렴 + 슬립(beta) 보정 시 진짜 차체헤딩 추종."""
    import random
    random.seed(3)
    sys = _make_sys(); e = sys.estimator
    e.x[:] = 0.0
    for _ in range(80):                        # 북향(math 90°) 잡음 COG
        e.predict(0.05)
        e.update_cog(math.radians(90.0) + random.gauss(0, 0.05), math.radians(3.0))
    hd = math.degrees(e.x[2]) % 360.0
    print(f"  잡음 COG 수렴 → {hd:.1f}° (기대 90°)")
    assert abs((hd - 90 + 180) % 360 - 180) < 5.0, f"COG 수렴 실패: {hd}"
    # 슬립: course = heading + beta. beta 를 주면 EKF 가 진짜 heading 추종
    e.x[:] = 0.0; e.P = e.np.eye(5) * 0.1
    TRUE = math.radians(90.0); BETA = math.radians(8.0)
    for _ in range(120):
        e.update_cog(TRUE + BETA, math.radians(1.0), beta_rad=BETA)
    hd2 = math.degrees(e.x[2]) % 360.0
    print(f"  슬립 8° 보정 후 → {hd2:.1f}° (진짜 차체헤딩 90°, 진로각 98° 아님)")
    assert abs((hd2 - 90 + 180) % 360 - 180) < 2.0, f"슬립 보정 실패: {hd2}"


def test_offset_convergence():
    """
    횡오프셋에서 라인 복귀 수렴 — 크로스트랙 '부호' 회귀 가드.
    (이 테스트 부재로 Stanley 부호반전이 그동안 미검출됐음.)
    ★ implement 는 작업기(후방점) 비최소위상 불안정으로 오프셋 발산 → 별도 이슈, 여기서 제외.
    """
    import sitl_sim
    for algo in ("pure_pursuit", "stanley"):
        sys_ = sitl_sim.build_system(algo=algo, profile="normal", realistic=True)
        sim = sitl_sim.Simulator(sys_, sitl_sim.KUBOTA_MR1157,
                                 target_speed=1.2, yaw_tau=0.2)
        sim.model.x = 0.8                       # 라인(x=0)에서 0.8m 횡오프셋
        for _ in range(700):
            if not sim.step(0.02)["engaged"]:
                break
        print(f"  {algo:13s} 0.8m → {sim.model.x:+.3f}m")
        assert abs(sim.model.x) < 0.2, f"{algo} 라인 복귀 실패(부호 의심): {sim.model.x:+.2f}m"


def test_implement_ff_curve():
    """작업기 곡선 피드포워드 — 곡선서 작업기오차 < pure_pursuit, 직선서 라인복귀 유지."""
    import sitl_sim, statistics
    from autosteer_core import Waypoint, KUBOTA_MR1157
    def arc(R):
        return [Waypoint(R*math.sin(k*0.008), R*(1-math.cos(k*0.008)), 1.0, True, "w")
                for k in range(500)]
    def sx(px, py, path):
        bi = min(range(len(path)), key=lambda i: math.hypot(path[i].x-px, path[i].y-py))
        j = min(bi+1, len(path)-1); i0 = max(j-1, 0)
        ph = math.atan2(path[j].y-path[i0].y, path[j].x-path[i0].x)
        return math.sin(ph)*(px-path[bi].x) - math.cos(ph)*(py-path[bi].y)
    def impl_err(algo, R):
        s = sitl_sim.build_system(algo=algo, profile="normal", realistic=True)
        s.set_algorithm(algo, s.params.wheelbase); s.path = arc(R)
        sim = sitl_sim.Simulator(s, KUBOTA_MR1157, target_speed=1.0, yaw_tau=0.2)
        sim.model.x = sim.model.y = sim.model.heading = 0.0; e = []
        for _ in range(1500):
            sim.step(0.02)
            ip = s.params.implement_position(sim.model.x, sim.model.y, sim.model.heading)
            e.append(abs(sx(ip[0], ip[1], s.path)))
        return statistics.mean(e[500:1300])
    base = impl_err("pure_pursuit", 8.0)
    ff   = impl_err("implement_ff", 8.0)
    print(f"  곡선(R8) 작업기오차: pure_pursuit {base:.4f}m → implement_ff {ff:.4f}m ({(1-ff/base)*100:+.0f}%)")
    assert ff < base * 0.7, f"implement_ff 곡선 개선 실패: {ff:.4f} vs {base:.4f}"
    # 직선 오프셋 복귀(FF=0 → pure_pursuit 동일)
    s2 = sitl_sim.build_system(algo="implement_ff", profile="normal", realistic=True)
    s2.set_algorithm("implement_ff", s2.params.wheelbase)
    sim2 = sitl_sim.Simulator(s2, sitl_sim.KUBOTA_MR1157, target_speed=1.2, yaw_tau=0.2)
    sim2.model.x = 0.8
    for _ in range(700):
        if not sim2.step(0.02)["engaged"]: break
    print(f"  직선 0.8m → {sim2.model.x:+.3f}m (라인복귀)")
    assert abs(sim2.model.x) < 0.2, f"implement_ff 직선 복귀 실패: {sim2.model.x:+.2f}m"


def test_ubx_cfg_builder():
    """UBX-CFG 빌더 — 체크섬 정합 + 무빙베이스 시퀀스에 RELPOSNED 활성 포함."""
    import f9p_client as fc
    # build_ubx 체크섬 자기정합: parse_relposned 가 같은 검증로직으로 통과해야 함
    f = fc.build_ubx(0x06, 0x01, bytes([0x01, 0x3C, 1]))
    assert f[:2] == b"\xb5\x62" and f[2] == 0x06 and f[3] == 0x01, "CFG-MSG 헤더 오류"
    cka, ckb = fc._ubx_checksum(f[2:-2])
    assert (cka, ckb) == (f[-2], f[-1]), "Fletcher 체크섬 불일치"
    # CFG-RATE 100ms
    r = fc.ubx_cfg_rate(100, 1, 1)
    assert r[2] == 0x06 and r[3] == 0x08 and r[6] | (r[7] << 8) == 100, "CFG-RATE payload 오류"
    # 무빙베이스 시퀀스: NAV-RELPOSNED(0x01,0x3C) 활성(rate=1) 프레임이 있어야 함
    seq = fc.moving_base_heading_cfg()
    # CFG-MSG 프레임 = B5 62 06 01 03 00 [cls id rate] ck_a ck_b = 11바이트
    has_relpos = any(m[2] == 0x06 and m[3] == 0x01 and len(m) == 11 and
                     m[6] == 0x01 and m[7] == 0x3C and m[8] == 1 for m in seq)
    has_save = any(m[2] == 0x06 and m[3] == 0x09 for m in seq)
    assert has_relpos, "무빙베이스 시퀀스에 RELPOSNED 활성 없음"
    assert has_save, "CFG-CFG 저장 프레임 없음"
    # 모든 프레임 체크섬 정합
    for m in seq:
        ca, cb = fc._ubx_checksum(m[2:-2])
        assert (ca, cb) == (m[-2], m[-1]), "시퀀스 프레임 체크섬 오류"
    print(f"  build_ubx 체크섬 OK · 무빙베이스 {len(seq)}프레임(RELPOSNED 활성·저장 포함)")


def test_scan_ports_safe():
    """포트 스캔 — 존재하지 않는 포트에도 안전(크래시/예외 없음)."""
    import f9p_client as fc
    out = fc.scan_ports(ports=["/dev/does-not-exist-xyz"], bauds=(115200,), window=0.05)
    assert out["best"] is None and out["ports"][0]["found"] is False, "미존재 포트 처리 실패"
    print("  미존재 포트 안전 처리 OK (best=None)")


if __name__ == "__main__":
    print("[1] HDT 나침반→수학각 변환")
    test_heading_convention()
    print("[2] GGA 위치 추종")
    test_position_tracking()
    print("[3] ver1 듀얼안테나+IMU 융합(평활)")
    test_dual_imu_fusion()
    print("[4] ver1 헤딩 바이어스 캘리브(20m 직선)")
    test_heading_calibration()
    print("[5] 무빙베이스 RELPOSNED 파싱(방법 1)")
    test_parse_relposned()
    print("[6] 에폭별 적응형 R(방법 3)")
    test_adaptive_heading_R()
    print("[7] fix 게이팅(방법 3)")
    test_heading_gating()
    print("[8] 베이스라인 틸트 보상 + roll 추정(방법 4)")
    test_tilt_compensation()
    print("[9] 듀얼 마운트 오프셋(base=좌/rover=우, +90°)")
    test_dual_mount_offset()
    print("[10] 듀얼 마운트 현장 진단 루틴(base/rover·부호 추천)")
    test_mount_diagnostic()
    print("[11] 진로각(COG) 보조 + 슬립 보정(방법 5)")
    test_cog_aiding()
    print("[12] 오프셋 라인복귀 수렴(크로스트랙 부호 가드)")
    test_offset_convergence()
    print("[13] UBX-CFG 빌더 + 무빙베이스 heading 활성 시퀀스")
    test_ubx_cfg_builder()
    print("[14] 내부 UART 포트 스캔(안전성)")
    test_scan_ports_safe()
    print("[15] 작업기 곡선 피드포워드(implement_ff) — 곡선 작업기오차 감소 + 직선 무영향")
    test_implement_ff_curve()
    print("\n  ✓ GNSS(NMEA/UBX)→EKF 입력 경로 검증 통과 — 헤딩 변환/위치 추종/무IMU predict "
          "+ 무빙베이스 헤딩(적응형R·게이팅·틸트)·COG보조. (조향 수렴은 sitl_sim 6/6)")
