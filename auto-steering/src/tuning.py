"""
tuning.py
=========
SITL 폐루프 위에서 TuningProfile 게인을 자동 탐색.

배경: sitl_sim 의 현실적 플랜트(rate-limit 서보 + 작업기 부하 yaw 지연)에서
AgNav 사진값 그대로의 PROFILE_HEAVY(k_cross=100, ptime=1.3)가 진동했다.
실제로 heavy 모드 고게인이 안정적인 건 '무거운 작업기 부하 감쇠' 덕인데,
모델 부하(yaw_tau)와 모터 응답이 정해지면 그에 맞는 게인을 정량 탐색할 수 있다.

⚠ 여기서 찾은 값은 '모델 기준' 추천치다. 실모터 응답(can_tools 로 계측)을
   ServoCanInterface 파라미터에 반영한 뒤 재탐색해야 현장값이 된다.

비용 = XTE_RMS(cm) + (미안착 시 큰 패널티). 안착 후보 중 최소 XTE 선택.
"""

from __future__ import annotations
import json
import random
import dataclasses
from dataclasses import dataclass
from typing import List, Optional

from autosteer_core import TuningProfile, PROFILE_HEAVY, KUBOTA_MR1157
from sitl_sim import build_system, Simulator, SimResult


def evaluate(profile: TuningProfile,
             yaw_tau: float = 0.6,
             dist_amp: float = 0.015,
             target_speed: float = 1.2,
             servo_rate_deg_s: float = 35.0,
             servo_tau: float = 0.08,
             steps: int = 500,
             seed: int = 3) -> SimResult:
    """주어진 프로파일을 현실적 플랜트(서보지연+작업기부하+횡외란)에서 폐루프 평가."""
    random.seed(seed)
    sys_ = build_system(KUBOTA_MR1157, algo="implement", profile=profile,
                        realistic=True, servo_rate_deg_s=servo_rate_deg_s,
                        servo_tau=servo_tau)
    sim = Simulator(sys_, KUBOTA_MR1157, target_speed=target_speed,
                    yaw_tau=yaw_tau, dist_amp=dist_amp)
    return sim.run(steps=steps)


def cost(res: SimResult) -> float:
    """안착 못 하면(진동/발산) 큰 패널티. 안착 후보는 XTE RMS 로 줄세움."""
    penalty = 0.0 if res.settled else 500.0
    return res.xte_rms_cm + 5.0 * res.reversal_rate * (0 if res.settled else 1) + penalty


@dataclass
class TuneResult:
    profile: TuningProfile
    res: SimResult
    cost: float


def tune_profile(base: TuningProfile = PROFILE_HEAVY,
                 yaw_tau: float = 0.6,
                 k_cross_grid=(20, 40, 60, 80, 100),
                 ptime_grid=(1.0, 1.5, 2.0, 2.5, 3.0),
                 k_heading_grid=(60, 100),
                 **eval_kw) -> List[TuneResult]:
    """그리드 탐색. 비용 오름차순 TuneResult 리스트 반환(첫 항목이 최적)."""
    out: List[TuneResult] = []
    for kc in k_cross_grid:
        for pt in ptime_grid:
            for kh in k_heading_grid:
                prof = dataclasses.replace(
                    base, name=f"{base.name}~tuned(kc{kc},pt{pt},kh{kh})",
                    k_cross=float(kc), ptime_on=float(pt), k_heading=float(kh))
                res = evaluate(prof, yaw_tau=yaw_tau, **eval_kw)
                out.append(TuneResult(prof, res, cost(res)))
    out.sort(key=lambda t: t.cost)
    return out


def save_profile(profile: TuningProfile, path: str, meta: Optional[dict] = None):
    d = {"_warning": "SITL 모델 기준 추천치 — 실모터 계측 반영 후 재검증 필요",
         "profile": dataclasses.asdict(profile)}
    if meta:
        d["_meta"] = meta
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=2, ensure_ascii=False)


# ── 자체 테스트: heavy 프로파일을 작업기 부하 하에서 자동 튜닝 ───────────
if __name__ == "__main__":
    import logging, tempfile, os
    logging.basicConfig(level=logging.ERROR)

    YAW_TAU = 0.6     # 과부하(쟁기/균평) 작업기 부하
    print("=" * 74)
    print(f"tuning — PROFILE_HEAVY 자동 튜닝 (작업기 부하 yaw_tau={YAW_TAU}s)")
    print("=" * 74)

    base = evaluate(PROFILE_HEAVY, yaw_tau=YAW_TAU)
    print(f"\n[기준] AgNav 사진값 heavy (k_cross=100, ptime=1.3, k_heading=100)")
    print(f"  XTE RMS {base.xte_rms_cm:.1f}cm  MAX {base.xte_max_cm:.1f}cm  "
          f"진동/s {base.reversal_rate:.1f}  안착 {'OK' if base.settled else '✗'}  "
          f"→ cost {cost(base):.1f}")

    print("\n그리드 탐색 중 (k_cross×ptime×k_heading = 5×5×2 = 50 조합)...")
    ranked = tune_profile(yaw_tau=YAW_TAU)

    print(f"\n[상위 5개 후보]")
    print(f"  {'k_cross':>7} {'ptime':>6} {'k_head':>6}  {'XTE RMS':>8}  "
          f"{'XTE MAX':>8}  {'안착':>4}  {'cost':>7}")
    print("  " + "-" * 58)
    for t in ranked[:5]:
        p, r = t.profile, t.res
        print(f"  {p.k_cross:>7.0f} {p.ptime_on:>6.1f} {p.k_heading:>6.0f}  "
              f"{r.xte_rms_cm:>6.1f}cm  {r.xte_max_cm:>6.1f}cm  "
              f"{'OK' if r.settled else '✗':>4}  {t.cost:>7.1f}")

    best = ranked[0]
    bp, br = best.profile, best.res
    print(f"\n[추천] k_cross={bp.k_cross:.0f}, ptime_on={bp.ptime_on:.1f}, "
          f"k_heading={bp.k_heading:.0f}")
    print(f"  기준 XTE {base.xte_rms_cm:.1f}cm(✗진동) → 추천 {br.xte_rms_cm:.1f}cm"
          f"({'안착' if br.settled else '✗'}),  cost {cost(base):.0f} → {best.cost:.1f}")
    print("\n  ※ 해석: 이 모델(서보 35°/s·지연 80ms)에서는 AgNav 사진값 k_cross=100 이")
    print("     서보 지연과 맞물려 진동한다. 예측 리드(ptime) 권한이 부족해 '저게인'이")
    print("     최적으로 나온다. 실모터가 더 빠르고 강성이면(can_tools 계측) 고게인이")
    print("     최적이 될 수 있으므로, 실측 서보 파라미터로 재탐색이 필수.")

    out_path = os.path.join(tempfile.gettempdir(), "heavy_tuned.json")
    save_profile(bp, out_path, meta={"yaw_tau": YAW_TAU,
                                     "baseline_xte_rms": base.xte_rms_cm,
                                     "tuned_xte_rms": br.xte_rms_cm})
    print(f"\n  추천 프로파일 저장: {out_path}")

    assert best.res.settled, "튜닝 후보가 안착하지 못함 — 그리드/플랜트 점검"
    assert best.cost < cost(base), "튜닝이 기준보다 개선되지 않음"
    assert br.xte_rms_cm < base.xte_rms_cm
    print("\n  ✓ 작업기 부하 모델에서 heavy 진동을 잡는 게인 자동 도출")
    print("  ✓ 실모터 응답을 ServoCanInterface(rate/tau)에 반영 후 재실행하면 현장값")
    os.remove(out_path)
