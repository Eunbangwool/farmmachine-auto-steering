"""
field_collect.py
================
현장 데이터 자동 수집 오케스트레이터.

실측(줄자) + 자동수집(주행/센서/CAN)을 하나의 세션으로 묶어:
  1) 각 수집 단계를 실행하고
  2) TractorParams / CanSpec / servo 파라미터에 자동 주입하고
  3) tractor.json + 세션 리포트(md) 로 저장한다.

설계: 각 stage_* 는 "이미 수집된 데이터"를 받는 순수 함수형(오프라인 테스트 가능),
collect_*_live 는 실장비 인터페이스에서 데이터를 모으는 얇은 래퍼.

수집 항목 ↔ 도구:
  GNSS 포맷/보레이트/RTK   ← f9p_client.GnssSniffer
  IMU 오프셋(파라미터5)    ← autosteer_core.ImuCalibrator (평지 30s)
  wheelbase, antenna_to_axle ← calibration (저속 사인주행)
  앵글센서 CAN ID/바이트    ← can_tools.correlate_with_signal (운전대 흔들기)
  모터 servo(max_rate,tau)  ← can_tools.MotorResponseProbe (스텝응답)
  나머지 치수(높이/작업기/트랙폭/최대WAS) ← 줄자/사진 (수동 입력)

자세한 측정법은 auto-steering/MEASUREMENT.md 참고.
"""

from __future__ import annotations
import json
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Callable

from autosteer_core import (KUBOTA_MR1157, TractorParams, ImuOffset,
                            ImuCalibrator, CanSpec)
from field_config import (save_config, apply_canspec, tractor_to_dict,
                          tractor_from_dict)
from calibration import estimate_from_log, Estimate
from can_tools import (correlate_with_signal, CanBusAnalyzer, CanFrame,
                       ServoResponse, MotorResponseProbe)

# 물리적으로 타당한 값 범위 (sanity check)
SANITY = {
    "wheelbase":        (1.5, 4.0),
    "antenna_to_axle":  (-2.0, 2.0),
    "antenna_height":   (1.5, 3.5),
    "antenna_to_impl":  (0.0, 4.0),
    "hitch_to_impl":    (0.0, 3.0),
    "front_track_width":(1.0, 2.5),
    "max_was_deg":      (10.0, 50.0),
}


@dataclass
class CollectionSession:
    params: TractorParams
    servo: Dict[str, float] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)
    collected: Dict[str, dict] = field(default_factory=dict)   # 항목별 결과/출처

    def _record(self, key: str, value, source: str, ok: bool, extra: str = ""):
        rng = SANITY.get(key)
        in_range = (rng is None) or (rng[0] <= value <= rng[1]
                                     if isinstance(value, (int, float)) else True)
        self.collected[key] = {"value": value, "source": source,
                               "ok": ok and in_range,
                               "in_range": in_range, "note": extra}

    def report_md(self) -> str:
        L = ["# 현장 데이터 수집 리포트",
             f"- 생성: {time.strftime('%Y-%m-%d %H:%M:%S')}", "",
             "| 항목 | 값 | 출처 | 상태 |", "|---|---|---|---|"]
        for k, v in self.collected.items():
            val = v["value"]
            vs = f"{val:.3f}" if isinstance(val, float) else str(val)
            st = "✅" if v["ok"] else "⚠️"
            note = f" ({v['note']})" if v["note"] else ""
            L.append(f"| {k} | {vs} | {v['source']} | {st}{note} |")
        if self.servo:
            L += ["", f"**servo**: {self.servo} (→ ServoCanInterface / tuning.py)"]
        if self.notes:
            L += ["", "## 메모"] + [f"- {n}" for n in self.notes]
        todo = [k for k, v in self.collected.items() if not v["ok"]]
        if todo:
            L += ["", "## ⚠️ 재확인 필요", *[f"- {k}" for k in todo]]
        return "\n".join(L)


class FieldDataCollector:
    """수집 세션. stage_*(데이터 주입) 또는 collect_*_live(실장비)로 채운다."""
    def __init__(self, params: Optional[TractorParams] = None):
        base = params or KUBOTA_MR1157
        # 전역 KUBOTA_MR1157 을 건드리지 않도록 복제
        self.session = CollectionSession(params=tractor_from_dict(tractor_to_dict(base)))

    @property
    def params(self) -> TractorParams:
        return self.session.params

    # ── 1) GNSS ────────────────────────────────────────────────
    def stage_gnss(self, sniff_summary: dict, baud: int, label: str = "pa3"):
        formats = sniff_summary.get("formats", [])
        q = sniff_summary.get("gga_qualities", [])
        rtk = 4 in q or 5 in q
        self.session._record(f"gnss_{label}", f"{baud}bps {','.join(formats)}",
                             "GnssSniffer", "nmea" in formats and rtk,
                             "RTK fix OK" if rtk else "RTK fix 미확인")

    # ── 2) IMU 오프셋 (파라미터 5) ──────────────────────────────
    def stage_imu(self, samples: List[Tuple[float, float, float]],
                  heading_ref_rad: Optional[float] = None):
        cal = ImuCalibrator(min_samples=1, min_duration=0.0)
        cal.start()
        for r, p, y in samples:
            cal.add_sample(r, p, y)
        off = cal.finish(heading_ref_rad=heading_ref_rad)
        self.params.imu_offset = off
        self.session._record("imu_offset",
                             f"r{off.roll:+.3f} p{off.pitch:+.3f} y{off.yaw:+.3f}",
                             "ImuCalibrator", True, f"{len(samples)}샘플")

    # ── 3) 운동학 자동추정 (wheelbase, antenna_to_axle) ─────────
    def stage_kinematics(self, drive_log: List[dict], apply: bool = True):
        res = estimate_from_log(drive_log)
        for key in ("wheelbase", "antenna_to_axle"):
            est: Estimate = res[key]
            if apply and est.ok:
                setattr(self.params, key, round(est.value, 3))
            self.session._record(key, round(est.value, 3), "calibration(주행)",
                                 est.ok, f"R²={est.r2:.2f} n={est.n_samples}")
        return res

    # ── 4) 앵글센서 CAN ID/바이트 (correlate) ───────────────────
    def stage_can_angle(self, frames: List[CanFrame],
                        signal: List[Tuple[float, float]]):
        hits = correlate_with_signal(frames, signal)
        if not hits:
            self.session._record("SENSOR_ANGLE_ID", "미검출", "correlate", False,
                                 "운전대 흔들기 신호/캡처 확인")
            return None
        h = hits[0]
        apply_canspec({"SENSOR_ANGLE_ID": h.can_id,
                       "SENSOR_BYTE_HI": h.hi, "SENSOR_BYTE_LO": h.lo,
                       "SENSOR_SIGNED": h.signed})
        polarity = "정상" if h.corr > 0 else "부호반전(스케일 −)"
        self.session._record("SENSOR_ANGLE_ID", f"0x{h.can_id:X}", "correlate",
                             abs(h.corr) > 0.8,
                             f"byte[{h.hi},{h.lo}] corr={h.corr:+.2f} {polarity}")
        return h

    # ── 5) 모터 servo (max_rate, tau) ───────────────────────────
    def stage_motor(self, response: ServoResponse):
        self.session.servo = response.as_servo_params()
        ok = response.max_rate_deg_s > 1.0
        self.session._record("servo", self.session.servo, "MotorResponseProbe",
                             ok, response.note or "스텝응답")

    # ── 6) 수동 실측 입력 (줄자/사진) ───────────────────────────
    def stage_manual(self, **kwargs):
        for k, v in kwargs.items():
            if v is None:
                continue
            # 자동추정과 교차검산
            prev = self.session.collected.get(k)
            note = "줄자"
            if prev and prev["source"].startswith("calibration") and isinstance(v, (int, float)):
                diff = abs(v - prev["value"])
                note = f"줄자, 자동추정과 Δ{diff:.2f}m"
            setattr(self.params, k, v)
            self.session._record(k, v, "manual", True, note)

    # ── 저장 ───────────────────────────────────────────────────
    def finalize(self, config_path: str, report_path: Optional[str] = None) -> str:
        save_config(config_path, self.params, include_canspec=True)
        rep = self.session.report_md()
        if report_path:
            with open(report_path, "w", encoding="utf-8") as f:
                f.write(rep)
        return rep

    # ── 실장비 라이브 래퍼 (하드웨어에서만) ─────────────────────
    def collect_gnss_live(self, port: str, baud: int = 115200,
                          duration: float = 5.0, label: str = "pa3"):
        from f9p_client import GnssSniffer
        rep = GnssSniffer(port, baud).run(duration)
        self.stage_gnss(rep.summary(), baud, label)

    def collect_imu_live(self, read_rpy: Callable[[], Tuple[float, float, float]],
                         duration: float = 30.0, hz: float = 100.0,
                         heading_ref_rad: Optional[float] = None):
        samples, t0, dt = [], time.time(), 1.0 / hz
        while time.time() - t0 < duration:
            samples.append(read_rpy()); time.sleep(dt)
        self.stage_imu(samples, heading_ref_rad)

    def collect_motor_live(self, can, **kw):
        probe = MotorResponseProbe(can, live=True)
        self.stage_motor(probe.measure(**kw))


# 현장 1일차 절차 안내
PROCEDURE = """\
[현장 1일차 데이터 수집 절차]  (자세한 측정법: MEASUREMENT.md)
 0. 안전: 데드맨/비상정지 동작 먼저 확인. 모터 계측은 바퀴 잭업 또는 빈 농지.
 1. 줄자 실측 → stage_manual(antenna_height=, antenna_to_impl=, hitch_to_impl=,
      front_track_width=, max_was_deg=)  [+ wheelbase 교차검산용]
 2. GNSS: collect_gnss_live("/dev/ttyS1",115200,'pa3') / (...,38400,'f9p')
 3. IMU: 평지 정지 → collect_imu_live(read_rpy, 30s)
 4. 운동학: 저속(1km/h) 좌우 사인주행 30초 로그 → stage_kinematics(drive_log)
 5. 앵글센서: 모터OFF, 운전대 좌우로 흔들며 CAN 캡처 + 흔든각도 신호 →
      stage_can_angle(frames, signal)
 6. 모터 servo: collect_motor_live(can)  → max_rate/tau
 7. finalize("tractor.json","collect_report.md")
 8. tuning.py 를 servo 파라미터로 재실행 → heavy 게인 확정
 9. sitl_sim.py 재검증 → 1km/h 실주행
"""


# ── 자체 테스트: 전 파이프라인을 합성 데이터로 끝까지 실행 ──────────────
if __name__ == "__main__":
    import math, random, tempfile, os
    random.seed(11)
    print("=" * 74)
    print("field_collect — 현장 수집 전 파이프라인 (합성 데이터) 검증")
    print("=" * 74)

    fc = FieldDataCollector()

    # (1) GNSS sniff 요약 (PA-3 NMEA + RTK fix)
    fc.stage_gnss({"formats": ["nmea"], "gga_qualities": [4, 5]}, 115200, "pa3")

    # (2) IMU: 평지 정지 30초치 (오프셋 roll=0.02, pitch=-0.01)
    imu = [(0.02 + random.gauss(0, 8e-4), -0.01 + random.gauss(0, 8e-4),
            math.pi / 2 + random.gauss(0, 2e-3)) for _ in range(300)]
    fc.stage_imu(imu, heading_ref_rad=math.pi / 2)

    # (3) 운동학: 알려진 L=2.55, d=-0.42 합성 저속 사인주행
    TRUE_L, TRUE_D, dt, v = 2.55, -0.42, 0.05, 1.2
    drive, x, y, hdg = [], 0.0, 0.0, math.pi / 2
    for k in range(1500):
        steer = math.radians(20) * math.sin(k * 0.02)
        yaw = v * math.tan(steer) / TRUE_L
        x += v * math.cos(hdg) * dt; y += v * math.sin(hdg) * dt
        hdg = math.atan2(math.sin(hdg + yaw * dt), math.cos(hdg + yaw * dt))
        ax = x + TRUE_D * math.cos(hdg) + random.gauss(0, 0.003)
        ay = y + TRUE_D * math.sin(hdg) + random.gauss(0, 0.003)
        drive.append(dict(x=ax, y=ay, heading=hdg + random.gauss(0, 0.002),
                          yaw_rate=yaw, speed=v, steer_rad=steer))
    fc.stage_kinematics(drive)

    # (4) 앵글센서 CAN: 0x18FF51E5 byte[2,3] 가 운전대 신호와 상관
    ANGLE_ID = 0x18FF51E5
    frames, signal, t = [], [], 0.0
    for k in range(800):
        t += 0.01
        deg = 20.0 * math.sin(k * 0.05); signal.append((t, deg))
        d = bytearray(8); d[2:4] = int(int(deg * 10) & 0xFFFF).to_bytes(2, "big")
        frames.append(CanFrame(t, ANGLE_ID, bytes(d)))
        d2 = bytearray(8); d2[0] = 1; frames.append(CanFrame(t, 0x18FF50E5, bytes(d2)))
    fc.stage_can_angle(frames, signal)

    # (5) 모터 servo 스텝응답 (실모터 대역 = ServoCanInterface)
    from sitl_sim import ServoCanInterface
    fc.stage_motor(MotorResponseProbe(ServoCanInterface(35.0, 0.08)).measure())

    # (6) 줄자 실측 입력
    fc.stage_manual(antenna_height=2.73, antenna_to_impl=1.20, hitch_to_impl=1.00,
                    front_track_width=1.56, max_was_deg=25.0, wheelbase=2.56)

    cfg = os.path.join(tempfile.gettempdir(), "tractor_collected.json")
    rep = os.path.join(tempfile.gettempdir(), "collect_report.md")
    print("\n" + fc.finalize(cfg, rep))

    # 검증: config 파일이 수집값을 담고 CanSpec 이 반영됐는지
    with open(cfg, encoding="utf-8") as f:
        saved = json.load(f)
    assert abs(saved["tractor"]["antenna_to_axle"] - TRUE_D) < 0.15
    assert saved["tractor"]["wheelbase"] == 2.56          # 수동 우선 적용
    assert saved["canspec"]["SENSOR_ANGLE_ID"] == ANGLE_ID
    assert "servo_rate_deg_s" in fc.session.servo
    assert isinstance(saved["tractor"]["imu_offset"], dict)
    print(f"\n  ✓ 전 단계 수집→config({os.path.basename(cfg)})+리포트 자동 생성")
    print("  ✓ CanSpec(앵글센서 ID) 반영, servo 파라미터 확보 → tuning 재탐색 준비완료")
    os.remove(cfg); os.remove(rep)
