# A Tabletop Mobile Robot for CT-Optimal Touch

Resources for the paper: *A Tabletop Mobile Robot for CT-Optimal Touch: Force-Controlled Stroking via Feedforward-PID Control*

## Contents

| Directory | Description | Status |
|-----------|-------------|--------|
| `control/` | Robot control software (holonomic base, IMU heading PID, FF+PID force control, calibration) | Coming soon |
| `data/` | Experiment raw data ($n = 20$): force recordings, EDA signals, questionnaire responses | Available |
| `cad/` | CAD files for robot chassis, stroking module, parallel-linkage mechanism | Available |
| `3d_scan/` | 20 participants' forearm 3D surface scans (OBJ + USDZ) | Pending |

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

**Reproducing the analysis:**

```bash
cd data/eda
pip install numpy scipy
python analyze_eda.py
```

This reads all CSVs in `rawdata/` and outputs `eda_features_reproduced.csv`. The script has been verified to produce identical results to `eda_features.csv` (532/532 values matched).

---

### Force (`data/force/`)

Force data from the stroking experiment. Target force: 0.4 N. Sampling rate: ~100 ms.

#### Master CSV (`master/`)

20 files: `S{NN}_master.csv`. Each contains all 4 conditions (2 directions × 2 controllers) with per-sample data.

| Column | Description | Unit |
|--------|-------------|------|
| `participant` | Participant ID (S01–S20) | |
| `direction` | `parallel` or `orthogonal` | |
| `control` | `ff_pid` (FF+PID) or `pid_only` (PID-only) | |
| `cycle` | Cycle number (1–10) | |
| `stroke` | `f` (forward), `b` (backward), `s` (stop/turnaround) | |
| `time_ms` | Raw timestamp | ms |
| `force_g` | Raw force | grams |
| `force_N` | Force in Newtons (`force_g × 0.00981`) | N |
| `pos_y_cm` | Y position (stroking direction) | cm |
| `pos_x_cm` | X position (perpendicular) | cm |
| `angle_deg` | Motor angle | degrees |
| `speed_cms` | Stroking speed | cm/s |
| `label_moving` | 1 = position changing, 0 = stationary | |
| `label_steady` | 1 = constant-velocity segment (analyzed), 0 = excluded | |

The three-stage filtering pipeline (see project page) uses `stroke`, `label_moving`, and `label_steady` to extract steady-state force samples. Only rows where `label_steady = 1` are used for metric computation.

#### Per-participant metrics (`ral_statistics.csv`)

80 rows (20 participants × 4 conditions). One row per participant × direction × control.

| Column | Description |
|--------|-------------|
| `mean` | Mean force (N) |
| `rmse` | RMSE vs 0.4 N target (N) |
| `mae` | Mean absolute error (N) |
| `cv` | Coefficient of variation (%) |
| `delta_f` | Forward–backward hysteresis (N) |
| `avg_delta_mean` | MASD — smoothness metric (N) |

#### Condition summary (`ral_summary.csv`)

4 rows (2 directions × 2 controls). Group-level descriptive statistics (M ± SD across 20 participants).

#### Advanced analysis (`advanced/`)

| File | Description |
|------|-------------|
| `cycle_trend.csv` | Spearman ρ (cycle vs RMSE) per participant × condition |
| `variance_decomposition.csv` | Between-participant vs within-participant RMSE variability |
| `icc_results.csv` | ICC(3,k) for 10-cycle RMSE reliability |
| `paired_tests.csv` | FF+PID vs PID-only paired comparisons per direction |

#### Metric definitions

All metrics are computed over steady-state data (`label_steady = 1`) for each participant–condition pair (cycles 1–10 pooled). Target force $F_{\text{target}} = 0.4\,\text{N}$.

| Metric | Formula | Description |
|--------|---------|-------------|
| Mean | $\frac{1}{n}\sum_{i=1}^{n} F_i$ | Average force during stroking |
| RMSE | $\sqrt{\frac{1}{n}\sum_{i=1}^{n}(F_i - F_{\text{target}})^2}$ | Root mean squared error vs target |
| MAE | $\frac{1}{n}\sum_{i=1}^{n}\|F_i - F_{\text{target}}\|$ | Mean absolute error vs target |
| CV | $\frac{\text{SD}(F)}{\overline{F}} \times 100$ | Coefficient of variation (%) |
| ΔF | $\|\overline{F}_{\text{fwd}} - \overline{F}_{\text{bwd}}\|$ | Forward–backward hysteresis |
| MASD | $\frac{1}{K}\sum_{k=1}^{K}\left(\frac{1}{n_k-1}\sum_{i=2}^{n_k}\|F_i^{(k)} - F_{i-1}^{(k)}\|\right)$ | Smoothness (per-cycle, then averaged) |

**Reproducing the analysis:**

```bash
cd data/force
python analyze_force.py
```

No external dependencies required (Python 3.8+ standard library only). This reads all master CSVs and outputs `ral_statistics_reproduced.csv`. The script has been verified to produce identical results to `ral_statistics.csv` (1040/1040 values matched).

---

## CAD

3D CAD files for the tabletop mobile robot, designed in Autodesk Inventor.

- `.iam` — Assembly files
- `.ipt` — Part files
- `.dwg` / `.pdf` — 2D drawings

| Directory | Description |
|-----------|-------------|
| `MOBITOUCH.iam`, `Assy.iam` | Top-level robot assembly |
| `mobile_robot/` | Holonomic mobile base: enclosure, lid, omniwheels, motor housings, odometry sensor, OpenCM controller board |
| `stroke_module/` | Parallel-linkage stroking module: link arms, gear assemblies, load cell (SC616C), motor adapter, shafts, spacers |
| `field/` | Experiment field: SUS430 steel plate, surface paper, protective barriers, markers |
| `emergency_stop/` | Emergency stop switch enclosure and circuit board |
| `jig.ipt` | Assembly/calibration jig |
| `plathome_usbhub.ipt` | USB hub platform |

**Note on Japanese filenames:** Some part files use Japanese names to maintain Autodesk Inventor assembly references. Translations: `平歯車` = spur gear, `Φ` = diameter.

**Hardware references:**
- Motors: Dynamixel XL-320 (×3)
- Load cell: SC616C
- Odometry: SparkFun Qwiic OTOS
- Controller: OpenCM 9.04

---

## 3D Forearm Scans (`3d_scan/`)

**Status: Pending.** Raw 3D scans (OBJ with textures, USDZ) of all 20 participants' forearms have been collected using Polycam (LiDAR). The scans require trimming (removing background geometry) before release. Analysis direction (e.g., forearm curvature vs calibration quality, simulation contact modeling) is under development. Data will be released in a future update.

## Project Page

See the [project page](https://tabletop-touch-project.netlify.app/) for detailed supplementary materials including force metrics, measurement reliability, EDA analysis, and calibration algorithms.
