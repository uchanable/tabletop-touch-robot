# MOBITOUCH Serial Logger - Usage Guide

## Overview

What the Python script handles automatically:
- `#define` mode switching (FORCE_PROFILING ↔ INTEGRATED_CONTROL)
- Serial monitor (replaces the Arduino IDE serial monitor)
- Calibration data parsing + automatic patching of integrated_control.ino
- Automatic CSV export of [LOG] data during the main experiment

What the user must do manually:
- Click the upload button in the Arduino IDE
- Hardware operations (board reset, external power ON/OFF, stroking module positioning)

---

## 0. Initial Setup (One-Time Only)

### Folder Structure

```
20_Experiment_RA-L/
  Arduino/              ← Existing (no changes)
  scripts/              ← Place scripts here
    mobitouch_serial.py
    requirements.txt
  Data/                 ← Auto-generated
```

### Installation (Windows)

```bash
pip install -r scripts/requirements.txt
```

---

## 1. Full Session for One Participant — `session` Command (Recommended)

Follows a pre-generated experiment order (CSV) to guide through 6 steps sequentially:

```bash
python scripts/mobitouch_serial.py session --subject S01
```

### Preparation: Generate Experiment Order (One-Time Only)

```bash
python scripts/mobitouch_serial.py generate-order
```

→ Creates `Data/experiment_order.csv` (S01–S20, counterbalanced)

- Odd-numbered participants: parallel block → orthogonal block
- Even-numbered participants: orthogonal block → parallel block
- PID / non-PID order within each block: randomized

### Session Flow

```
┌─────────────────────────────────────────────────┐
│  Session: S01 (6 steps, auto-guided)            │
├─────────────────────────────────────────────────┤
│                                                 │
│  [Pre-session] 3D Arm Scan (Polycam LiDAR)     │
│    → Scan forearm → Save .obj (~3 min)          │
│                                                 │
│  Step 1/6: calibrate parallel                   │
│    → Same as `run` command (calibration+patch)  │
│                                                 │
│  Step 2/6: record parallel_PID                  │
│    → Same as `record` command                   │
│                                                 │
│  Step 3/6: record parallel                      │
│    → Same as `record` command                   │
│                                                 │
│  ⚠ Direction Change (parallel → orthogonal)     │
│    "Change the stroking module orientation"      │
│    → Loosen bolts → Rotate → Tighten bolts      │
│                                                 │
│  Step 4/6: calibrate orthogonal                 │
│    → Same as `run` command (calibration+patch)  │
│                                                 │
│  Step 5/6: record orthogonal_PID                │
│    → Same as `record` command                   │
│                                                 │
│  Step 6/6: record orthogonal                    │
│    → Same as `record` command                   │
│                                                 │
│  Session complete!                              │
└─────────────────────────────────────────────────┘
```

---

## 2. Parallel / Orthogonal Conditions — `run` Command

Runs the full pipeline from calibration through the main experiment in a single command:

```bash
python scripts/mobitouch_serial.py run --condition parallel --subject S01
```

### Execution Flow

```
┌─────────────────────────────────────────────────┐
│  PHASE 1: Calibration                           │
├─────────────────────────────────────────────────┤
│                                                 │
│  [Auto] #define → MODE_FORCE_PROFILING          │
│                                                 │
│  [Prompt] "Upload from Arduino IDE"             │
│  [User]                                         │
│    1. Upload from Arduino IDE                   │
│    2. Reset the board                           │
│    3. Adjust stroking module height → position  │
│    4. External power ON                         │
│    5. Press Enter                               │
│                                                 │
│  [Serial monitor starts]                        │
│    - Type p → Static scan begins                │
│    - Static scan completes                      │
│    - Return device to home position             │
│      (no reset/power cycle needed)              │
│    - Press Enter → Dynamic learning pass 1      │
│    - (Return to home → Enter) × 4 more passes   │
│    - Pass 5 complete → Data output automatic    │
│                                                 │
│  [Auto] Trajectory data parsing                 │
│  [Auto] integrated_control.ino patching         │
│  [Auto] Learning data saved to CSV              │
│  Press Ctrl+C to close the serial monitor       │
│                                                 │
├─────────────────────────────────────────────────┤
│  PHASE 2: Main Experiment                       │
├─────────────────────────────────────────────────┤
│                                                 │
│  [Auto] #define → MODE_INTEGRATED_CONTROL       │
│                                                 │
│  [Prompt] "Upload again from Arduino IDE"       │
│  [User]                                         │
│    1. External power OFF                        │
│    2. Reset the board                           │
│    3. Upload from Arduino IDE                   │
│    4. Adjust stroking module height → position  │
│    5. External power ON                         │
│    6. Press Enter                               │
│                                                 │
│  [Serial monitor starts]                        │
│    - Type all start                             │
│    - Set up participant's arm                   │
│    - Press space + Enter                        │
│      → ★ Simultaneously press EDA marker button │
│    - Stroking begins after ~15 seconds          │
│    - [LOG] data auto-saved to CSV               │
│    - "=== 往復完了 ===" → Recording complete     │
│    - External power OFF                         │
│    - Press Ctrl+C to exit                       │
│                                                 │
├─────────────────────────────────────────────────┤
│  [Auto] #define → MODE_FORCE_PROFILING          │
│         (reverted for the next participant)     │
│  Session complete!                              │
└─────────────────────────────────────────────────┘
```

---

## 3. PID Conditions (parallel_PID, orthogonal_PID) — `record` Command

Runs the main experiment directly without calibration:

```bash
python scripts/mobitouch_serial.py record --condition parallel_PID --subject S01
```

### Pre-Execution Setup (Manual)

1. Upload `MOBITOUCH_parallel_PID.ino` from the Arduino IDE
2. Reset the board
3. Adjust stroking module height → position in place
4. External power ON
5. Run the script

### Execution Flow

```
[Serial monitor starts]
  - Type all start
  - Set up participant's arm
  - Press space + Enter → ★ Simultaneously press EDA marker button
  - Stroking begins after ~15 seconds
  - [LOG] data auto-saved to CSV
  - "=== 往復完了 ===" → Recording complete
  - External power OFF
  - Press Ctrl+C to exit
```

---

## 4. Full Session for One Participant — Manual vs. Automated

### Method A: `session` Command (Recommended)

```bash
# With a pre-generated experiment order CSV, one command guides all 6 steps
python scripts/mobitouch_serial.py session --subject S01
```

### Method B: Individual Commands (Manual)

```bash
# 1. parallel — Calibration + main experiment
python scripts/mobitouch_serial.py run --condition parallel --subject S01

# 2. parallel_PID — Main experiment only
#    (After uploading MOBITOUCH_parallel_PID.ino from Arduino IDE)
python scripts/mobitouch_serial.py record --condition parallel_PID --subject S01

# 3. orthogonal — Calibration + main experiment
python scripts/mobitouch_serial.py run --condition orthogonal --subject S01

# 4. orthogonal_PID — Main experiment only
#    (After uploading MOBITOUCH_orthogonal_PID.ino from Arduino IDE)
python scripts/mobitouch_serial.py record --condition orthogonal_PID --subject S01
```

---

## Output File Structure

```
Data/
  calibration/
    parallel/
      20260308_143052_raw.log        ← Calibration serial log
      20260308_143052_learning.csv   ← Learning data (static/dynamic)
  parallel/
    S01/
      20260308_150230.csv            ← Main experiment data (processed)
      20260308_150230_raw.log        ← Main experiment serial log (raw)
  parallel_PID/
    S01/
      ...
  orthogonal/
    S01/
      ...
  orthogonal_PID/
    S01/
      ...
```

---

## Command Summary

| Command | Purpose | Target Conditions |
|---------|---------|-------------------|
| `session --subject Y` | Full session for one participant (6 steps, auto-guided) | Per CSV order |
| `generate-order` | Generate experiment order CSV (S01–S20) | — |
| `run --condition X --subject Y` | Calibration + main experiment (fully automated) | parallel, orthogonal |
| `record --condition X --subject Y` | Main experiment only | All 4 conditions |
| `calibrate --condition X --subject Y` | Calibration only | parallel, orthogonal |
| `ports` | List available serial ports | — |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Port not found` / `Cannot open port` | Close the Arduino IDE serial monitor and retry |
| Auto-detection of port fails | Specify the port manually with `--port COM7` |
| CSV contains no data | Ensure the script is running before typing `all start` |
| Compilation error after patching | Restore from the `.ino.bak` backup |
| `#define` change not applied | If the `.ino` file is open in Arduino IDE, save and reload it |
