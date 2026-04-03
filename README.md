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

---

### EDA (`data/eda/`)

Electrodermal activity recorded using a BITalino (r)evolution board at 10 Hz. Ag/AgCl electrodes on the left index and middle fingertips.

> **Note:** Participant S18 is included in the data but was excluded from all statistical analyses due to near-zero SCL values (0.47–0.65 µS vs group mean ~9–10 µS), attributable to dry skin or poor electrode contact.

#### Raw time series (`rawdata/`)

80 CSV files: `{participant}_{direction}_{control}.csv`

Examples: `S01_parallel_FFPID.csv`, `S05_orthogonal_PID.csv`

| Column | Description | Unit |
|--------|-------------|------|
| `time_s` | Time relative to stroking onset | seconds (0.0 = stroking start) |
| `eda_uS` | EDA signal (skin conductance) | µS (microSiemens) |
| `phase` | Window phase | `pre` (−20 s), `stroking`, `post` (+20 s) |

Each file contains the stroking window with ±20 s pre/post margins. Sampling rate: 10 Hz.

**How the measurement window is defined:**

1. The experimenter simultaneously presses the EDA sync button and the sequence start key.
2. The delay from sequence start to first robot motion (~14 s) plus the stroking duration is added to the button-press timestamp to determine the analysis window.
3. ±20 s margins are included for context.

The precise window boundaries (button press time, stroking start/end samples) are documented in `eda_button_matching.csv`.

#### Button matching (`eda_button_matching.csv`)

Maps each EDA recording to the experimental event timing.

| Column | Description |
|--------|-------------|
| `participant` | Participant ID |
| `direction` | `parallel` or `orthogonal` |
| `control` | `FF+PID` or `PID` |
| `eda_file` | Source BITalino filename |
| `eda_button_index` | Which button press in the EDA file corresponds to this condition |
| `eda_button_time` | Clock time of button press |
| `eda_button_sample` | Sample index of button press in raw EDA file |
| `eda_stroking_start_sample` | Sample index where stroking begins |
| `eda_stroking_end_sample` | Sample index where stroking ends |
| `stroking_delay_s` | Delay from button press to stroking start (s) |
| `stroking_duration_s` | Total stroking duration (s) |
| `match_quality` | `GOOD` or annotation of issues |

#### Extracted features (`eda_features.csv`)

76 rows (S18 excluded from analysis, but data is present). One row per participant × direction × control.

**SCL (tonic) features:**

| Column | Description | Unit |
|--------|-------------|------|
| `scl_mean` | Mean skin conductance during stroking | µS |
| `scl_sd` | Standard deviation | µS |
| `scl_range` | Max − Min | µS |
| `scl_auc` | Area under the SCL curve (trapezoidal integration) | µS·s |

**SCR (phasic) features:**

| Column | Description | Unit |
|--------|-------------|------|
| `scr_amplitude` | Mean peak amplitude of detected SCR events | µS |
| `scr_count` | Number of SCR peaks detected | count |
| `scr_auc_baseline` | Area under phasic signal, baseline-corrected (5 s pre-stroking mean) | µS·s |

**Signal processing for SCR extraction:**

The phasic component is obtained by applying a 2nd-order Butterworth bandpass filter (0.05–3.0 Hz) with zero-phase distortion (`filtfilt`). Peak detection is then performed on the filtered signal to identify individual SCR events. The baseline is defined as the mean SCR level during the 5 s immediately preceding stroking onset.

## Project Page

See the [project page](https://tabletop-touch-project.netlify.app/) for detailed supplementary materials including force metrics, measurement reliability, EDA analysis, and calibration algorithms.
