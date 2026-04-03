#!/usr/bin/env python3
"""Reproduce EDA feature extraction from raw time-series CSVs.

Reads the per-session CSVs in rawdata/ and computes SCL (tonic) and SCR (phasic)
features, matching the pipeline described in the supplementary materials.

Requirements:
    pip install numpy scipy

Usage:
    python analyze_eda.py

Output:
    eda_features_reproduced.csv  (compare with eda_features.csv)
"""

import csv
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

# ── Constants ──
SR = 10  # Sampling rate (Hz)
RAWDATA_DIR = Path(__file__).parent / "rawdata"
OUTPUT_FILE = Path(__file__).parent / "eda_features_reproduced.csv"

# SCR bandpass filter parameters
SCR_LOWCUT = 0.05   # Hz
SCR_HIGHCUT = 3.0   # Hz
SCR_ORDER = 2       # Butterworth filter order
SCR_BASELINE_S = 5  # Seconds before stroking onset for baseline


def load_rawdata(filepath: Path) -> tuple[np.ndarray, int, int]:
    """Load a raw EDA CSV and return (signal, stroking_start_idx, stroking_end_idx)."""
    times, values, phases = [], [], []
    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time_s"]))
            values.append(float(row["eda_uS"]))
            phases.append(row["phase"])

    signal = np.array(values)

    # Find stroking window indices
    stroking_start = None
    stroking_end = None
    for i, phase in enumerate(phases):
        if phase == "stroking" and stroking_start is None:
            stroking_start = i
        if phase == "post" and stroking_end is None:
            stroking_end = i

    if stroking_start is None:
        stroking_start = 0
    if stroking_end is None:
        stroking_end = len(signal)

    return signal, stroking_start, stroking_end


def bandpass_filter(signal: np.ndarray) -> np.ndarray:
    """2nd-order Butterworth bandpass filter (0.05–3.0 Hz), zero-phase."""
    nyq = 0.5 * SR
    b, a = butter(SCR_ORDER, [SCR_LOWCUT / nyq, SCR_HIGHCUT / nyq], btype="band")
    return filtfilt(b, a, signal)


def compute_scl(segment: np.ndarray) -> dict:
    """SCL (tonic) features from the stroking segment."""
    if len(segment) < 3:
        return {}
    return {
        "scl_mean": round(float(np.mean(segment)), 4),
        "scl_sd": round(float(np.std(segment, ddof=1)), 4),
        "scl_min": round(float(np.min(segment)), 4),
        "scl_max": round(float(np.max(segment)), 4),
        "scl_range": round(float(np.max(segment) - np.min(segment)), 4),
        "scl_auc": round(float(np.trapezoid(segment, dx=1.0 / SR)), 4),
        "n_samples": len(segment),
        "duration_s": round(len(segment) / SR, 1),
    }


def compute_scr(full_window: np.ndarray, stk_start: int, stk_end: int) -> dict:
    """SCR (phasic) features using bandpass-filtered signal."""
    if len(full_window) < SR * 5:
        return {}

    filtered = bandpass_filter(full_window)

    # Baseline: -10s to -5s before stroking start
    bl_start = max(0, stk_start - int(10 * SR))
    bl_end = max(0, stk_start - int(SCR_BASELINE_S * SR))
    if bl_end <= bl_start:
        bl_end = stk_start
    baseline_data = filtered[bl_start:bl_end]
    baseline_mean = float(np.mean(baseline_data)) if len(baseline_data) > 0 else 0.0

    # Stimulus window
    stim = filtered[stk_start:stk_end]
    if len(stim) < 3:
        return {}

    # Peak detection
    min_height = baseline_mean + 0.01
    try:
        peaks, _ = find_peaks(stim, height=min_height)
    except Exception:
        peaks = np.array([])

    peak_idx = peaks[np.argmax(stim[peaks])] if len(peaks) > 0 else int(np.argmax(stim))
    peak_value = float(stim[peak_idx])
    amplitude = peak_value - baseline_mean

    # Latency
    threshold = baseline_mean + 0.01
    above = np.where(stim > threshold)[0]
    latency_s = float(above[0] / SR) if len(above) > 0 else 0.0

    # AUC
    dx = 1.0 / SR
    auc_zero = float(np.trapezoid(stim, dx=dx))
    auc_baseline = float(np.trapezoid(stim - baseline_mean, dx=dx))
    auc_initial = float(np.trapezoid(stim - float(stim[0]), dx=dx))

    # SCR count
    try:
        all_peaks, _ = find_peaks(stim, height=min_height, distance=int(SR * 1))
        scr_count = len(all_peaks)
    except Exception:
        scr_count = 0

    return {
        "scr_baseline_mean": round(baseline_mean, 4),
        "scr_peak_value": round(peak_value, 4),
        "scr_amplitude": round(amplitude, 4),
        "scr_latency_s": round(latency_s, 2),
        "scr_peak_time_s": round(peak_idx / SR, 2),
        "scr_count": scr_count,
        "scr_auc_zero": round(auc_zero, 4),
        "scr_auc_baseline": round(auc_baseline, 4),
        "scr_auc_initial": round(auc_initial, 4),
    }


def parse_filename(name: str) -> tuple[str, str, str]:
    """Parse 'S01_parallel_FFPID.csv' → ('S01', 'parallel', 'FF+PID')."""
    stem = name.replace(".csv", "")
    parts = stem.split("_")
    participant = parts[0]
    direction = parts[1]
    control = "FF+PID" if parts[2] == "FFPID" else "PID"
    return participant, direction, control


def main():
    csv_files = sorted(RAWDATA_DIR.glob("*.csv"))
    print(f"Found {len(csv_files)} raw EDA files")

    results = []
    for fpath in csv_files:
        participant, direction, control = parse_filename(fpath.name)
        signal, stk_start, stk_end = load_rawdata(fpath)
        stroking_segment = signal[stk_start:stk_end]

        scl = compute_scl(stroking_segment)
        scr = compute_scr(signal, stk_start, stk_end)

        row = {
            "participant": participant,
            "direction": direction,
            "control": control,
            "stroking_duration_s": round((stk_end - stk_start) / SR, 1),
            **scl,
            **scr,
        }
        results.append(row)
        print(f"  {participant} {direction:10s} {control:6s}  SCL={scl.get('scl_mean','N/A')}  SCR_count={scr.get('scr_count','N/A')}")

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
