# A Tabletop Mobile Robot for CT-Optimal Touch

Resources for the paper: *A Tabletop Mobile Robot for CT-Optimal Touch: Force-Controlled Stroking via Feedforward-PID Control*

## Contents

| Directory | Description | Status |
|-----------|-------------|--------|
| `control/` | Robot control software (holonomic base, IMU heading PID, FF+PID force control, calibration) | Coming soon |
| `data/` | Experiment raw data ($n = 20$): force recordings, EDA signals, questionnaire responses | Available |
| `cad/` | CAD files for robot chassis, stroking module, parallel-linkage mechanism | Coming soon |
| `3d_scan/` | 20 participants' forearm 3D surface scans (OBJ format) | Coming soon |

## Data

### Questionnaire (`data/questionnaire/`)

After each 10-cycle stroking condition, participants rated the tactile experience on three 7-point Likert scales (1 = not at all, 7 = very much):

| Scale | Japanese | Question |
|-------|----------|----------|
| Comfort | 快適 (kaiteki) | "The stroking sensation felt pleasant" |
| Naturalness | 自然 (shizen) | "The stroking felt natural" |
| Consistency | 一定 (ittei) | "The force felt consistent during stroking" |

**File:** `questionnaire.csv`

| Column | Description |
|--------|-------------|
| `participant` | Participant ID (S01–S20) |
| `order` | Presentation order (1–4) |
| `condition` | Condition label |
| `direction` | Stroking direction: `parallel` or `orthogonal` |
| `control` | Controller type: `ff_pid` (FF+PID) or `pid_only` (PID-only) |
| `comfort` | Comfort rating (1–7) |
| `naturalness` | Naturalness rating (1–7) |
| `consistency` | Consistency rating (1–7) |

Each participant provided 4 responses (2 directions × 2 controllers), yielding 80 rows total.

## Project Page

See the [project page](https://tabletop-touch-project.netlify.app/) for detailed supplementary materials including force metrics, measurement reliability, EDA analysis, and calibration algorithms.
