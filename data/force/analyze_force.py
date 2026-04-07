#!/usr/bin/env python3
"""Reproduce force metric computation from master CSVs.

Reads the per-participant master CSVs in master/ and computes all six force
metrics reported in the paper, matching the pipeline described in the
supplementary materials.

Data selection:
  - All metrics (Mean, SD, RMSE, MAE, CV, MASD, per-cycle RMSE):
    ALL rows within cycles 1-10 are used. No filtering applied.
  - Hysteresis (delta_f = |mean(F_fwd) - mean(F_bwd)|):
    Only rows where stroke in ("f", "b") AND label_moving == 1.
    This excludes turnaround stops but retains acceleration/deceleration.

Metrics:
  Mean   = (1/n) * sum(F_i)
  RMSE   = sqrt((1/n) * sum((F_i - F_target)^2))
  MAE    = (1/n) * sum(|F_i - F_target|)
  CV     = SD(F) / mean(F) * 100
  dF     = |mean(F_fwd) - mean(F_bwd)|   [moving-phase data only]
  MASD   = (1/K) * sum_k( (1/(n_k-1)) * sum_i(|F_i - F_{i-1}|) )

Requirements:
    Python 3.8+ (no external dependencies)

Usage:
    python analyze_force.py

Output:
    ral_statistics_reproduced.csv  (compare with ral_statistics.csv)
"""

import csv
import math
from pathlib import Path

# -- Constants --
MASTER_DIR = Path(__file__).parent / "master"
OUTPUT_FILE = Path(__file__).parent / "ral_statistics_reproduced.csv"
TARGET_FORCE_N = 0.4
MAX_CYCLE = 10


def load_master(filepath: Path) -> list[dict]:
    with open(filepath, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def compute_mean(forces: list[float]) -> float:
    """Mean = (1/n) * sum(F_i)"""
    return sum(forces) / len(forces)


def compute_rmse(forces: list[float], target: float) -> float:
    """RMSE = sqrt( (1/n) * sum( (F_i - F_target)^2 ) )"""
    return math.sqrt(sum((f - target) ** 2 for f in forces) / len(forces))


def compute_mae(forces: list[float], target: float) -> float:
    """MAE = (1/n) * sum( |F_i - F_target| )"""
    return sum(abs(f - target) for f in forces) / len(forces)


def compute_cv(forces: list[float]) -> float:
    """CV = SD(F) / mean(F) * 100"""
    n = len(forces)
    mean_f = sum(forces) / n
    sd = math.sqrt(sum((f - mean_f) ** 2 for f in forces) / (n - 1)) if n > 1 else 0
    return (sd / mean_f * 100) if mean_f != 0 else 0


def compute_delta_f(rows: list[dict]) -> tuple[float, float, float, float]:
    """dF = |mean(F_forward) - mean(F_backward)|

    Computed from moving-phase data only (stroke in f/b, label_moving=1).
    Returns: (fwd_mean, bwd_mean, delta_f, delta_f_pct)
    """
    fwd = [float(r["force_N"]) for r in rows if r["stroke"] == "f"]
    bwd = [float(r["force_N"]) for r in rows if r["stroke"] == "b"]
    fwd_mean = sum(fwd) / len(fwd) if fwd else 0
    bwd_mean = sum(bwd) / len(bwd) if bwd else 0
    delta_f = abs(fwd_mean - bwd_mean)
    delta_f_pct = (delta_f / TARGET_FORCE_N * 100) if TARGET_FORCE_N != 0 else 0
    return fwd_mean, bwd_mean, delta_f, delta_f_pct


def compute_masd(rows: list[dict]) -> tuple[float, float]:
    """MASD = (1/K) * sum_k( (1/(n_k - 1)) * sum_i( |F_i - F_{i-1}| ) )

    Mean of adjacent-sample force changes per cycle, then averaged across
    K cycles. Lower values indicate temporally smoother force delivery.
    Returns: (masd_mean, masd_sd)
    """
    cycles = sorted(set(int(r["cycle"]) for r in rows))
    cycle_avg_deltas = []

    for c in cycles:
        if c > MAX_CYCLE:
            continue
        forces = [float(r["force_N"]) for r in rows if int(r["cycle"]) == c]
        if len(forces) < 2:
            continue
        deltas = [abs(forces[i] - forces[i - 1]) for i in range(1, len(forces))]
        cycle_avg_deltas.append(sum(deltas) / len(deltas))

    if not cycle_avg_deltas:
        return 0.0, 0.0

    mean_ad = sum(cycle_avg_deltas) / len(cycle_avg_deltas)
    sd_ad = (
        math.sqrt(
            sum((d - mean_ad) ** 2 for d in cycle_avg_deltas)
            / (len(cycle_avg_deltas) - 1)
        )
        if len(cycle_avg_deltas) > 1
        else 0
    )
    return mean_ad, sd_ad


def compute_per_cycle_rmse(rows: list[dict]) -> tuple[float, float, int]:
    """Per-cycle RMSE: compute RMSE for each cycle, then mean and SD.

    Returns: (cycle_rmse_mean, cycle_rmse_sd, n_cycles)
    """
    cycles = sorted(set(int(r["cycle"]) for r in rows))
    cycle_rmses = []

    for c in cycles:
        if c > MAX_CYCLE:
            continue
        forces = [float(r["force_N"]) for r in rows if int(r["cycle"]) == c]
        if forces:
            rmse = math.sqrt(sum((f - TARGET_FORCE_N) ** 2 for f in forces) / len(forces))
            cycle_rmses.append(rmse)

    if not cycle_rmses:
        return 0.0, 0.0, 0

    mean_rmse = sum(cycle_rmses) / len(cycle_rmses)
    sd_rmse = (
        math.sqrt(
            sum((r - mean_rmse) ** 2 for r in cycle_rmses) / (len(cycle_rmses) - 1)
        )
        if len(cycle_rmses) > 1
        else 0
    )
    return mean_rmse, sd_rmse, len(cycle_rmses)


def main():
    csv_files = sorted(MASTER_DIR.glob("S*_master.csv"))
    print(f"Found {len(csv_files)} master CSVs")

    directions = ["parallel", "orthogonal"]
    controls = ["ff_pid", "pid_only"]
    results = []

    for fpath in csv_files:
        participant = fpath.stem.replace("_master", "")
        rows = load_master(fpath)

        for direction in directions:
            for control in controls:
                # All data within cycles 1-10 (no filtering)
                all_cond_rows = [
                    r for r in rows
                    if r["direction"] == direction
                    and r["control"] == control
                    and int(r["cycle"]) <= MAX_CYCLE
                ]

                # Hysteresis only: stroke in (f, b) AND label_moving == 1
                # Excludes turnaround stops but keeps accel/decel
                moving_rows = [
                    r for r in rows
                    if r["direction"] == direction
                    and r["control"] == control
                    and int(r["cycle"]) <= MAX_CYCLE
                    and r["stroke"] in ("f", "b")
                    and int(r["label_moving"]) == 1
                ]

                if not all_cond_rows:
                    print(f"  {participant} {direction} {control}: NO DATA")
                    continue

                forces = [float(r["force_N"]) for r in all_cond_rows]
                n = len(forces)
                mean_f = compute_mean(forces)
                sd_f = math.sqrt(sum((f - mean_f) ** 2 for f in forces) / (n - 1)) if n > 1 else 0
                rmse = compute_rmse(forces, TARGET_FORCE_N)
                mae = compute_mae(forces, TARGET_FORCE_N)
                cv = compute_cv(forces)

                # Delta_f from moving-phase rows only
                fwd_mean, bwd_mean, delta_f, delta_f_pct = compute_delta_f(moving_rows)

                # MASD from all data (no filtering)
                masd_mean, masd_sd = compute_masd(all_cond_rows)

                # Per-cycle RMSE from all data (no filtering)
                cr_mean, cr_sd, n_cycles = compute_per_cycle_rmse(all_cond_rows)

                row = {
                    "participant": participant,
                    "direction": direction,
                    "control": control,
                    "condition": f"{direction}_{'FF+PID' if control == 'ff_pid' else 'PID'}",
                    "mean": round(mean_f, 5),
                    "sd": round(sd_f, 5),
                    "rmse": round(rmse, 5),
                    "mae": round(mae, 5),
                    "cv": round(cv, 2),
                    "n_points": n,
                    "cycle_rmse_mean": round(cr_mean, 5),
                    "cycle_rmse_sd": round(cr_sd, 5),
                    "n_cycles": n_cycles,
                    "fwd_mean": round(fwd_mean, 5),
                    "bwd_mean": round(bwd_mean, 5),
                    "delta_f": round(delta_f, 5),
                    "delta_f_pct": round(delta_f_pct, 2),
                    "avg_delta_mean": round(masd_mean, 6),
                    "avg_delta_sd": round(masd_sd, 6),
                }
                results.append(row)
                print(f"  {participant} {direction:10s} {control:8s}  RMSE={rmse:.4f}  MASD={masd_mean:.5f}")

    # Write output
    if results:
        fieldnames = list(results[0].keys())
        with open(OUTPUT_FILE, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print(f"\nSaved: {OUTPUT_FILE} ({len(results)} rows)")


if __name__ == "__main__":
    main()
