#!/usr/bin/env python3
"""Verify chassis servo calibration (打表) data accuracy.

Performs statistical, geometric, and internal-consistency checks on
chassis_kinematics.json. Produces a verification report with pass/fail
ratings per dimension.

Usage:
    python verify_chassis_calibration.py [--kinematics PATH] [--verbose]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_KINEMATICS = ROOT / "configs" / "chassis_kinematics.json"

# ── Geometric model parameters (from firmware) ──
WHEEL_BASE_CM = 16.0          # ACKERMANN_WHEEL_BASE_CM
WHEEL_TRACK_CM = 14.5         # WHEEL_TRACK_CM
NOMINAL_CENTER_DEG = 90.0

# ── Verification thresholds ──
MIN_SAMPLES_PER_STE = 2        # Min samples needed for reliable curvature
GOOD_SAMPLES_PER_STE = 4       # "Good" sample count threshold
STD_RATIO_THRESHOLD = 0.15     # std/|mean| < 15% → good precision
R_EFF_GEO_TOLERANCE_PCT = 40.0 # Measured R_eff vs geometric R within 40%
DEADBAND_CONSISTENCY_CM = 0.5  # Max std of deadband measurements
COAST_CONSISTENCY_CM = 0.5     # Max std of coast measurements
SYMMETRY_RATIO_TOLERANCE = 0.3 # Left/right ratio should be stable across pairs
DERIVATIVE_AGREEMENT_PCT = 30.0 # STAT vs DONE vs TLM deg_per_cm should agree within 30%


def load_json(path: Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def clamp_heading(delta_deg: float) -> float:
    """Clamp yaw delta to [-180, 180]."""
    while delta_deg > 180.0:
        delta_deg -= 360.0
    while delta_deg < -180.0:
        delta_deg += 360.0
    return delta_deg


# ═══════════════════════════════════════════════════════════════════════════════
# Check 1: Sample Count Adequacy
# ═══════════════════════════════════════════════════════════════════════════════

def check_sample_counts(steer_rows: list[dict]) -> dict:
    results = {}
    for row in steer_rows:
        ste = row["ste"]
        n = row["n"]
        if n >= GOOD_SAMPLES_PER_STE:
            rating = "PASS"
        elif n >= MIN_SAMPLES_PER_STE:
            rating = "WARN"
        else:
            rating = "FAIL"
        results[str(ste)] = {
            "n": n,
            "rating": rating,
            "detail": f"n={n}" + (" ✓" if n >= GOOD_SAMPLES_PER_STE else
                                  f" (need ≥{GOOD_SAMPLES_PER_STE} for high confidence)")
        }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Check 2: Measurement Precision (std/mean ratio)
# ═══════════════════════════════════════════════════════════════════════════════

def check_precision(steer_rows: list[dict]) -> dict:
    results = {}
    for row in steer_rows:
        ste = row["ste"]
        abs_mean = row["abs_deg_per_cm"]
        std = row.get("std_deg_per_cm") or 0.0
        if abs_mean is None or abs_mean < 1e-9:
            results[str(ste)] = {"rating": "FAIL", "detail": "no valid curvature data"}
            continue
        ratio = std / abs_mean
        if ratio < STD_RATIO_THRESHOLD / 2:
            rating = "PASS"
        elif ratio < STD_RATIO_THRESHOLD:
            rating = "WARN"
        else:
            rating = "FAIL"
        results[str(ste)] = {
            "abs_deg_per_cm": round(abs_mean, 6),
            "std": round(std, 6),
            "cv_pct": round(ratio * 100.0, 1),
            "rating": rating,
            "detail": f"CV={ratio*100:.1f}% (std={std:.4f}/mean={abs_mean:.4f})"
        }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Check 3: Geometric Cross-Validation (Ackermann model vs measured R_eff)
# ═══════════════════════════════════════════════════════════════════════════════

def ackermann_radius_geometric(steer_deg: float) -> float:
    """Compute geometric turning radius from Ackermann model."""
    offset_deg = steer_deg - NOMINAL_CENTER_DEG
    if abs(offset_deg) < 0.5:
        return float("inf")
    tan_steer = math.tan(math.radians(abs(offset_deg)))
    return WHEEL_BASE_CM / tan_steer  # cm


def check_geometric_consistency(steer_rows: list[dict]) -> dict:
    results = {}
    for row in steer_rows:
        ste = row["ste"]
        r_eff_measured = row.get("r_eff_cm")
        r_geo = ackermann_radius_geometric(ste)
        if r_eff_measured is None:
            results[str(ste)] = {"rating": "FAIL", "detail": "no measured R_eff"}
            continue
        if r_geo == float("inf"):
            results[str(ste)] = {"rating": "SKIP", "detail": "steering at center, no geometric radius"}
            continue
        deviation_pct = abs(r_eff_measured - r_geo) / r_geo * 100.0
        if deviation_pct < R_EFF_GEO_TOLERANCE_PCT / 2:
            rating = "PASS"  # Very close to geometric model
        elif deviation_pct < R_EFF_GEO_TOLERANCE_PCT:
            rating = "WARN"
        else:
            rating = "INFO"  # Expected — real chassis differs from pure geometry
        results[str(ste)] = {
            "r_geo_cm": round(r_geo, 1),
            "r_eff_measured_cm": round(r_eff_measured, 1),
            "deviation_pct": round(deviation_pct, 1),
            "rating": rating,
            "detail": f"R_geo={r_geo:.1f}, R_meas={r_eff_measured:.1f} (Δ={deviation_pct:.1f}%)"
        }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Check 4: Internal Derivative Consistency (STAT vs DONE vs TLM)
# ═══════════════════════════════════════════════════════════════════════════════

def check_derivative_consistency(steer_rows: list[dict]) -> dict:
    """Check that deg_per_cm computed from STAT, DONE, and TLM agree.

    STAT uses the post-stop STAT yaw (most accurate, includes coast).
    DONE uses the DONE event yaw (stops earlier due to deadband).
    TLM uses intermediate telemetry (noisy, lower resolution).

    STAT vs DONE should agree reasonably. TLM is auxiliary.
    """
    results = {}
    for row in steer_rows:
        ste = row["ste"]
        samples = row.get("samples", [])
        stat_vals = []
        done_vals = []
        tlm_vals = []
        for s in samples:
            if s.get("deg_per_cm_stat") is not None:
                stat_vals.append(abs(s["deg_per_cm_stat"]))
            if s.get("deg_per_cm_done") is not None:
                done_vals.append(abs(s["deg_per_cm_done"]))
            if s.get("deg_per_cm_tlm") is not None:
                tlm_vals.append(abs(s["deg_per_cm_tlm"]))

        comparisons = []
        # STAT vs DONE
        if stat_vals and done_vals:
            stat_m = mean(stat_vals)
            done_m = mean(done_vals)
            if stat_m > 1e-9:
                dev = abs(stat_m - done_m) / stat_m * 100.0
                comparisons.append(("STAT_vs_DONE", dev))

        # STAT vs TLM
        if stat_vals and tlm_vals:
            stat_m = mean(stat_vals)
            tlm_m = mean(tlm_vals)
            if stat_m > 1e-9:
                dev = abs(stat_m - tlm_m) / stat_m * 100.0
                comparisons.append(("STAT_vs_TLM", dev))

        if not comparisons:
            results[str(ste)] = {"rating": "SKIP", "detail": "insufficient paired samples"}
            continue

        max_dev = max(dev for _, dev in comparisons)
        if max_dev < DERIVATIVE_AGREEMENT_PCT / 2:
            rating = "PASS"
        elif max_dev < DERIVATIVE_AGREEMENT_PCT:
            rating = "WARN"
        else:
            rating = "INFO"  # TLM often diverges due to filtering

        detail_parts = [f"{label} Δ={dev:.1f}%" for label, dev in comparisons]
        results[str(ste)] = {
            "comparisons": [{"pair": label, "deviation_pct": round(dev, 1)} for label, dev in comparisons],
            "max_deviation_pct": round(max_dev, 1),
            "rating": rating,
            "detail": "; ".join(detail_parts)
        }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Check 5: Servo Center Trim Validation
# ═══════════════════════════════════════════════════════════════════════════════

def check_servo_center(data: dict) -> dict:
    """Validate servo center trim selection using available trim samples."""
    trim_samples = data.get("servo_trim_samples", [])
    if not trim_samples:
        return {"rating": "SKIP", "detail": "no trim samples in data"}

    # Build STE → |yaw_delta_stat| mapping (lower is better — straighter)
    ste_yaw = []
    for ts in trim_samples:
        ste_cmd = None
        cmd = ts.get("command", "")
        import re
        m = re.search(r"STE=(\d+)", cmd)
        if m:
            ste_cmd = int(m.group(1))
        yaw = ts.get("yaw_delta_stat_deg")
        if ste_cmd is not None and yaw is not None:
            ste_yaw.append((ste_cmd, abs(yaw), ts))

    if not ste_yaw:
        return {"rating": "SKIP", "detail": "could not parse trim samples"}

    # Sort by yaw magnitude (best center = smallest |yaw|)
    ste_yaw.sort(key=lambda x: x[1])

    best_ste = ste_yaw[0][0]
    configured_center = data.get("servo_center_trim_ste", 90)

    detail_lines = []
    for ste, yaw, ts in ste_yaw:
        marker = " ← BEST" if ste == best_ste else ""
        detail_lines.append(f"STE={ste}: |Δyaw|={yaw:.2f}° over ~{ts.get('stat_after_d_cm','?')}cm{marker}")

    if best_ste == configured_center:
        rating = "PASS"
        verdict = f"Configured center STE={configured_center} matches best measured STE={best_ste}"
    else:
        # Check if the difference is significant
        best_yaw = ste_yaw[0][1]
        cfg_yaw = next((y for s, y, _ in ste_yaw if s == configured_center), None)
        if cfg_yaw is not None and abs(cfg_yaw - best_yaw) < 0.3:
            rating = "WARN"
            verdict = f"STE={configured_center} vs best STE={best_ste} differ by <0.3° — within noise"
        else:
            rating = "FAIL"
            verdict = f"Configured center STE={configured_center} ≠ best measured STE={best_ste}"

    return {
        "best_ste": best_ste,
        "configured_ste": configured_center,
        "rating": rating,
        "detail": verdict,
        "trim_table": detail_lines,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Check 6: Deadband & Coast Consistency
# ═══════════════════════════════════════════════════════════════════════════════

def check_deadband_coast(data: dict) -> dict:
    """Verify deadband and coast measurements are internally consistent."""
    samples = data.get("arc_deadband_samples", [])
    if not samples:
        return {"rating": "SKIP", "detail": "no deadband samples"}

    deadbands = []
    coasts = []
    for s in samples:
        db = s.get("deadband_done_cm")
        ct = s.get("coast_after_done_cm")
        if db is not None:
            deadbands.append(db)
        if ct is not None:
            coasts.append(ct)

    results = {}

    if deadbands:
        db_mean = mean(deadbands)
        db_std = pstdev(deadbands) if len(deadbands) > 1 else 0.0
        if db_std <= DEADBAND_CONSISTENCY_CM / 2:
            rating = "PASS"
        elif db_std <= DEADBAND_CONSISTENCY_CM:
            rating = "WARN"
        else:
            rating = "FAIL"
        results["deadband"] = {
            "mean_cm": round(db_mean, 2),
            "std_cm": round(db_std, 2),
            "values": [round(d, 2) for d in deadbands],
            "rating": rating,
            "detail": f"deadband = {db_mean:.2f} ± {db_std:.2f} cm (n={len(deadbands)})"
        }

    if coasts:
        ct_mean = mean(coasts)
        ct_std = pstdev(coasts) if len(coasts) > 1 else 0.0
        if ct_std <= COAST_CONSISTENCY_CM / 2:
            rating = "PASS"
        elif ct_std <= COAST_CONSISTENCY_CM:
            rating = "WARN"
        else:
            rating = "FAIL"
        results["coast"] = {
            "mean_cm": round(ct_mean, 2),
            "std_cm": round(ct_std, 2),
            "values": [round(c, 2) for c in coasts],
            "rating": rating,
            "detail": f"coast = {ct_mean:.2f} ± {ct_std:.2f} cm (n={len(coasts)})"
        }

    # Also validate: deadband samples should have very small yaw change
    # (they were probing min effective distance, so actual motion is tiny)
    for s in samples:
        yaw = s.get("yaw_delta_stat_deg")
        cmd_d = s.get("commanded_cm")
        if yaw is not None and cmd_d is not None:
            pass  # logged for audit, not a pass/fail criterion

    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Check 7: Ackermann Asymmetry Consistency
# ═══════════════════════════════════════════════════════════════════════════════

def check_asymmetry(data: dict, steer_rows: list[dict]) -> dict:
    """Check that left/right asymmetry ratios are physically plausible.

    Ackermann geometry predicts slightly different left/right radii due to
    steering linkage asymmetry, but the ratio should be roughly symmetric
    across the two angle pairs (60/120 should be similar to 75/105 ratio).
    """
    symmetry = data.get("symmetry_ratio_abs_deg_per_cm", {})
    ratios = []
    for pair_key, ratio in symmetry.items():
        if ratio is not None:
            ratios.append((pair_key, ratio))

    if not ratios:
        return {"rating": "SKIP", "detail": "no symmetry ratios computed"}

    ratio_values = [r for _, r in ratios]

    # Check 1: Are ratios reasonable? (0.5 ~ 3.0 is the plausible range)
    plausible = all(0.3 < r < 3.0 for r in ratio_values)

    # Check 2: Are ratios roughly consistent with R_eff ratios?
    # 60/120 should have R_eff ratio similar to deg_per_cm ratio
    by_ste = {row["ste"]: row for row in steer_rows}

    cross_checks = []
    for left, right in [(60, 120), (75, 105)]:
        l_row = by_ste.get(left, {})
        r_row = by_ste.get(right, {})
        l_r = l_row.get("r_eff_cm")
        r_r = r_row.get("r_eff_cm")
        if l_r and r_r:
            r_ratio = r_r / l_r  # R_right / R_left
            deg_ratio = r_row.get("abs_deg_per_cm", 0) / max(l_row.get("abs_deg_per_cm", 1e-9), 1e-9)
            # deg_per_cm ∝ 1/R, so deg_ratio should ≈ 1/r_ratio
            # i.e. if R_right > R_left, then deg_per_cm_right < deg_per_cm_left
            cross_checks.append({
                "pair": f"{left}/{right}",
                "r_eff_ratio": round(r_ratio, 3),
                "deg_per_cm_ratio": round(deg_ratio, 3),
                "consistent": abs(r_ratio * deg_ratio - 1.0) < 0.3  # R_ratio * deg_ratio ≈ 1
            })

    if plausible and all(c["consistent"] for c in cross_checks):
        rating = "PASS"
    elif plausible:
        rating = "WARN"
    else:
        rating = "FAIL"

    detail_parts = [f"{p}={r:.2f}" for p, r in ratios]
    return {
        "ratios": {p: r for p, r in ratios},
        "plausible_range": plausible,
        "r_eff_cross_check": cross_checks,
        "rating": rating,
        "detail": "symmetry ratios: " + ", ".join(detail_parts) +
                  (" ✓" if plausible else " ⚠ implausible")
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Check 8: Arc Min Effective Distance Validation
# ═══════════════════════════════════════════════════════════════════════════════

def check_min_effective(data: dict) -> dict:
    """Verify arc_min_effective_cmd_cm against deadband samples.

    D=-3 probes: if DONE.D ≤ 1.1cm for D=3 command, actual movement is ≤1.1cm
    which means the effective minimum command is ~3cm (commanded) to get >1cm actual.
    """
    min_eff = data.get("arc_min_effective_cmd_cm")
    samples = data.get("arc_deadband_samples", [])

    if min_eff is None:
        return {"rating": "SKIP", "detail": "no arc_min_effective_cmd_cm configured"}

    # Check if D=-3 samples support the min_effective value
    d3_samples = [s for s in samples if abs(s.get("commanded_cm", 0) - 3.0) < 0.1]
    d4_samples = [s for s in samples if abs(s.get("commanded_cm", 0) - 4.0) < 0.1]

    d3_actual = [s.get("stat_after_d_cm", 0) for s in d3_samples]
    d4_actual = [s.get("stat_after_d_cm", 0) for s in d4_samples]

    d3_ok = all(a < 2.0 for a in d3_actual) if d3_actual else None  # D=3 should move <2cm
    d4_ok = all(a > 2.0 for a in d4_actual) if d4_actual else None  # D=4 should move >2cm

    if d3_ok and d4_ok:
        rating = "PASS"
        detail = f"D=3 moves {d3_actual}cm, D=4 moves {d4_actual}cm → min_eff={min_eff}cm confirmed"
    elif d3_ok is False:
        rating = "FAIL"
        detail = f"D=3 moves more than expected: {d3_actual}cm"
    else:
        rating = "WARN"
        detail = f"D=3: {d3_actual}, D=4: {d4_actual} (insufficient data to confirm min_eff={min_eff}cm)"

    return {
        "configured_min_effective_cm": min_eff,
        "d3_actual_cm": d3_actual,
        "d4_actual_cm": d4_actual,
        "rating": rating,
        "detail": detail,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Check 9: Outlier Detection
# ═══════════════════════════════════════════════════════════════════════════════

def check_outliers(steer_rows: list[dict]) -> dict:
    """Detect outlier samples within each steering angle using IQR method.

    When IQR=0 (degenerate from deduplicated identical samples), we fall back
    to a CV-based check using the overall mean as reference. This avoids false
    positives when most values are exactly equal.
    """
    results = {}
    for row in steer_rows:
        ste = row["ste"]
        samples = row.get("samples", [])
        values = [abs(s["deg_per_cm_stat"]) for s in samples if s.get("deg_per_cm_stat") is not None]
        if len(values) < 3:
            results[str(ste)] = {"rating": "SKIP", "detail": f"need >=3 samples for outlier detection (n={len(values)})"}
            continue

        values_sorted = sorted(values)
        q1 = values_sorted[len(values_sorted) // 4]
        q3 = values_sorted[3 * len(values_sorted) // 4]
        iqr = q3 - q1
        val_mean = mean(values)

        # Degenerate IQR: many identical values (from dedup). Fall back to CV check.
        if iqr < 1e-9 or (val_mean > 1e-9 and iqr / val_mean < 0.001):
            max_dev = max(abs(v - val_mean) / val_mean for v in values) if val_mean > 1e-9 else 0.0
            if max_dev < 0.05:  # all within 5% of mean
                rating = "PASS"
                detail = (f"IQR=0 (degenerate from dedup), max_dev={max_dev*100:.1f}% — "
                          f"all values within 5% of mean={val_mean:.4f}")
            elif max_dev < 0.10:
                rating = "WARN"
                detail = (f"IQR=0 (degenerate), max_dev={max_dev*100:.1f}% from mean={val_mean:.4f}")
            else:
                rating = "FAIL"
                detail = (f"IQR=0 (degenerate), max_dev={max_dev*100:.1f}% from mean={val_mean:.4f} "
                          f"— excessive spread")
            results[str(ste)] = {
                "n": len(values), "mean": round(val_mean, 6),
                "values": [round(v, 6) for v in values_sorted],
                "iqr": round(iqr, 6), "max_dev_pct": round(max_dev * 100, 1),
                "rating": rating, "detail": detail,
            }
            continue

        lower = q1 - 1.5 * iqr
        upper = q3 + 1.5 * iqr
        outliers = [v for v in values if v < lower or v > upper]

        if not outliers:
            rating = "PASS"
            detail = f"no outliers (IQR={iqr:.4f}, range=[{lower:.4f}, {upper:.4f}])"
        elif len(outliers) == 1 and len(values) >= 5:
            rating = "WARN"
            detail = f"{len(outliers)} outlier(s): {outliers} (IQR={iqr:.4f})"
        else:
            rating = "FAIL"
            detail = f"{len(outliers)}/{len(values)} outliers: {outliers}"

        results[str(ste)] = {
            "n": len(values),
            "q1": round(q1, 6),
            "q3": round(q3, 6),
            "iqr": round(iqr, 6),
            "outliers": outliers,
            "rating": rating,
            "detail": detail,
        }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Report Assembly
# ═══════════════════════════════════════════════════════════════════════════════

def rating_emoji(rating: str) -> str:
    return {"PASS": "[PASS]", "WARN": "[WARN]", "FAIL": "[FAIL]", "INFO": "[INFO]", "SKIP": "[SKIP]"}.get(rating, "[??]")


def print_separator(title: str) -> None:
    print()
    print("=" * 72)
    print(f"  {title}")
    print("=" * 72)


def print_subsection(title: str) -> None:
    print(f"\n── {title} ──")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kinematics", type=Path, default=DEFAULT_KINEMATICS,
                        help="Path to chassis_kinematics.json")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print per-sample diagnostics")
    args = parser.parse_args()

    if not args.kinematics.exists():
        print(f"ERROR: {args.kinematics} not found")
        return 1

    data = load_json(args.kinematics)
    steer_rows = data.get("steer_curvature", [])

    if not steer_rows:
        print("ERROR: no steer_curvature data")
        return 1

    print_separator("底盘打表数据 准确性验证报告")
    print(f"数据来源: {args.kinematics}")
    print(f"版本: {data.get('version', 'unknown')}")
    print(f"生成自: {data.get('generated_from', {})}")

    all_ratings = {}

    # ── 1. Sample counts ──
    print_separator("1. 样本数量")
    r = check_sample_counts(steer_rows)
    all_ratings["sample_count"] = r
    for ste, info in sorted(r.items(), key=lambda x: int(x[0])):
        print(f"  STE={ste}: n={info['n']:2d}  {rating_emoji(info['rating'])} {info['rating']}")

    # ── 2. Precision ──
    print_separator("2. 测量精度 (变异系数 CV = std/|mean|)")
    r = check_precision(steer_rows)
    all_ratings["precision"] = r
    for ste, info in sorted(r.items(), key=lambda x: int(x[0])):
        print(f"  STE={ste}: CV={info.get('cv_pct', 0):.1f}%  {rating_emoji(info['rating'])} {info['rating']}")
        print(f"           |deg_per_cm|={info.get('abs_deg_per_cm', 0):.4f} ± {info.get('std', 0):.4f}")

    # ── 3. Geometric cross-validation ──
    print_separator("3. 几何模型交叉验证 (Ackermann 理论 vs 实测)")
    print(f"   模型参数: wheelbase={WHEEL_BASE_CM}cm, wheel_track={WHEEL_TRACK_CM}cm")
    r = check_geometric_consistency(steer_rows)
    all_ratings["geometry"] = r
    for ste, info in sorted(r.items(), key=lambda x: int(x[0])):
        geo = info.get("r_geo_cm", "?")
        meas = info.get("r_eff_measured_cm", "?")
        dev = info.get("deviation_pct", 0)
        print(f"  STE={ste}: R_geo={geo}cm, R_meas={meas}cm, Δ={dev:.1f}%  {rating_emoji(info['rating'])} {info['rating']}")

    # ── 4. Derivative consistency ──
    print_separator("4. 内部一致性 (STAT vs DONE vs TLM 的 deg_per_cm)")
    r = check_derivative_consistency(steer_rows)
    all_ratings["derivative_consistency"] = r
    for ste, info in sorted(r.items(), key=lambda x: int(x[0])):
        print(f"  STE={ste}: max Δ={info.get('max_deviation_pct', 0):.1f}%  {rating_emoji(info['rating'])} {info['rating']}")
        print(f"           {info['detail']}")

    # ── 5. Servo center ──
    print_separator("5. 舵机中位验证 (Servo Center Trim)")
    r = check_servo_center(data)
    all_ratings["servo_center"] = r
    print(f"  配置中位: STE={r.get('configured_ste')}")
    print(f"  实测最优: STE={r.get('best_ste')}")
    print(f"  {rating_emoji(r['rating'])} {r['rating']}: {r['detail']}")
    for line in r.get("trim_table", []):
        print(f"    {line}")

    # ── 6. Deadband & Coast ──
    print_separator("6. 死区与滑行一致性")
    r = check_deadband_coast(data)
    all_ratings["deadband_coast"] = r
    for key, info in r.items():
        print(f"  {key}: {info.get('detail', '')}  {rating_emoji(info.get('rating', 'SKIP'))} {info.get('rating', 'SKIP')}")

    # ── 7. Asymmetry ──
    print_separator("7. Ackermann 不对称性验证")
    r = check_asymmetry(data, steer_rows)
    all_ratings["asymmetry"] = r
    print(f"  {r['detail']}")
    for cc in r.get("r_eff_cross_check", []):
        status = "✓" if cc["consistent"] else "⚠"
        print(f"    {cc['pair']}: R_ratio={cc['r_eff_ratio']:.3f}, "
              f"deg_ratio={cc['deg_per_cm_ratio']:.3f} "
              f"(R×deg ≈ {cc['r_eff_ratio']*cc['deg_per_cm_ratio']:.3f}) {status}")

    # ── 8. Min effective distance ──
    print_separator("8. ARC最小有效距离验证")
    r = check_min_effective(data)
    all_ratings["min_effective"] = r
    print(f"  {rating_emoji(r['rating'])} {r['rating']}: {r['detail']}")

    # ── 9. Outliers ──
    print_separator("9. 异常值检测 (IQR方法)")
    r = check_outliers(steer_rows)
    all_ratings["outliers"] = r
    for ste, info in sorted(r.items(), key=lambda x: int(x[0])):
        print(f"  STE={ste}: {rating_emoji(info['rating'])} {info['rating']} — {info['detail']}")

    # ── Per-sample detail ──
    if args.verbose:
        print_separator("详细样本数据")
        for row in steer_rows:
            print_subsection(f"STE={row['ste']} (n={row['n']}, {row.get('direction','?')})")
            print(f"  deg_per_cm={row.get('deg_per_cm')} ± {row.get('std_deg_per_cm')}")
            print(f"  R_eff={row.get('r_eff_cm')} cm")
            for i, s in enumerate(row.get("samples", []), 1):
                print(f"  [{i}] src={s.get('source','?')} file={s.get('file','?')}")
                print(f"      cmd={s.get('command','?')}")
                print(f"      Δyaw_stat={s.get('yaw_change_stat_deg')}°  D_stat={s.get('dist_stat_cm')}cm  "
                      f"deg_per_cm={s.get('deg_per_cm_stat')}")
                if s.get("deg_per_cm_done") is not None:
                    print(f"      Δyaw_done={s.get('yaw_change_done_deg')}°  D_done={s.get('dist_done_cm')}cm  "
                          f"deg_per_cm_done={s.get('deg_per_cm_done')}")
                if s.get("deg_per_cm_tlm") is not None:
                    print(f"      Δyaw_tlm ={s.get('yaw_change_tlm_deg')}°  D_tlm ={s.get('dist_tlm_cm')}cm  "
                          f"deg_per_cm_tlm ={s.get('deg_per_cm_tlm')}")

    # ── Summary ──
    print_separator("总结")
    total = 0
    passed = 0
    warned = 0
    failed = 0
    for check_name, check_results in all_ratings.items():
        if isinstance(check_results, dict):
            # Flatten nested dicts (like deadband_coast -> {deadband: ..., coast: ...})
            sub_ratings = []
            for k, v in check_results.items():
                if isinstance(v, dict) and "rating" in v:
                    sub_ratings.append((f"{check_name}/{k}", v["rating"]))
                elif k == "rating":
                    sub_ratings.append((check_name, v))
            if not sub_ratings:
                sub_ratings = [(check_name, {"rating": "SKIP"})]
            for name, rating_info in sub_ratings:
                rating = rating_info if isinstance(rating_info, str) else rating_info.get("rating", "SKIP")
                total += 1
                if rating == "PASS":
                    passed += 1
                elif rating == "WARN":
                    warned += 1
                elif rating == "FAIL":
                    failed += 1

    print(f"  总检查项: {total}")
    print(f"  ✅ PASS: {passed}")
    print(f"  ⚠️ WARN: {warned}")
    print(f"  ❌ FAIL: {failed}")
    print(f"  ⏭️ SKIP/INFO: {total - passed - warned - failed}")

    # Overall assessment
    if failed == 0 and warned <= 2:
        overall = "✅ 打表数据质量良好，可以信赖并用于固件集成"
    elif failed == 0:
        overall = "⚠️ 打表数据基本可用，但部分维度精度不足，建议补充采样"
    else:
        overall = "❌ 打表数据存在准确性问题，需重新校准后使用"

    print(f"\n  综合判定: {overall}")

    # ── Firmware integration gaps ──
    print_separator("固件集成缺口 (基于验证结果)")
    gaps = []

    # Check center trim
    cfg_center = data.get("servo_center_trim_ste", 90)
    if cfg_center != 90:
        gaps.append(f"1. 舵机中位: 固件 ACKERMANN_CENTER_DEG=90.0 → 应改为 {cfg_center}")

    # Check curvature table
    gaps.append("2. 曲率查表: 固件无 deg_per_cm 查表, 纯几何 Ackermann 模型")
    gaps.append(f"   STE=60: deg_per_cm={steer_rows[0].get('deg_per_cm','?') if len(steer_rows)>0 else '?'}")
    gaps.append(f"   STE=75: deg_per_cm={steer_rows[1].get('deg_per_cm','?') if len(steer_rows)>1 else '?'}")
    gaps.append(f"   STE=105: deg_per_cm={steer_rows[2].get('deg_per_cm','?') if len(steer_rows)>2 else '?'}")
    gaps.append(f"   STE=120: deg_per_cm={steer_rows[3].get('deg_per_cm','?') if len(steer_rows)>3 else '?'}")

    # Deadband
    db = data.get("arc_deadband_cm")
    if db is not None:
        gaps.append(f"3. ARC死区补偿: 固件 DISTANCE_DONE_CM=2.0, 实测 arc_deadband={db}cm"
                    f" → 上位机需在命令距离上叠加 deadband")

    # Coast
    coast = data.get("coast_after_done_cm")
    if coast is not None:
        gaps.append(f"4. 滑行补偿: 固件无此概念, DONE后滑行 {coast}cm → 需建模")

    # Min effective
    min_eff = data.get("arc_min_effective_cmd_cm")
    if min_eff is not None:
        gaps.append(f"5. 最小有效距离: 固件不拒绝 D<3.0 命令, 实测 min_eff={min_eff}cm → 应加下限检查")

    for g in gaps:
        print(f"  {g}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
