#!/usr/bin/env python3
"""
MOBITOUCH Serial Logger
- run:       calibrate → record fully automated flow (parallel/orthogonal)
- record:    main experiment serial log auto-save
- calibrate: calibration data auto-processing + .ino patch
- ports:     list available serial ports
"""

import argparse
import csv
import difflib
import os
import platform
import re
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

try:
    from colorama import Fore, Style, init as colorama_init
    colorama_init()
except ImportError:
    class _NoColor:
        def __getattr__(self, _):
            return ""
    Fore = Style = _NoColor()

# ============================================================
# Constants
# ============================================================
BASE_DIR = Path(__file__).resolve().parent.parent  # Arduino/
ARDUINO_DIR = BASE_DIR  # .ino folders are directly under BASE_DIR
DATA_DIR = BASE_DIR / "Data"

DEFAULT_BAUD = 115200
PORT_KEYWORDS = ["ROBOTIS", "OpenCR", "OpenCM"]

CONDITIONS_ALL = ["parallel", "parallel_PID", "orthogonal", "orthogonal_PID"]
CONDITIONS_CALIBRATE = ["parallel", "orthogonal"]

# condition → Arduino folder name
CONDITION_ARDUINO_MAP = {
    "parallel": "MOBITOUCH_parallel",
    "parallel_PID": "MOBITOUCH_parallel_PID",
    "orthogonal": "MOBITOUCH_orthogonal",
    "orthogonal_PID": "MOBITOUCH_orthogonal_PID",
}

CSV_HEADER = "time_ms,force_g,pos_y_m,pos_x_m,angle_deg,speed_cms"


# ============================================================
# Utility Functions
# ============================================================

def find_port(preferred=None):
    """Find serial port - prefer specified, then auto-detect."""
    ports = serial.tools.list_ports.comports()
    if preferred:
        for p in ports:
            if p.device == preferred:
                return p.device
        print(f"{Fore.RED}[ERROR] Port {preferred} not found.{Style.RESET_ALL}")
        _show_ports(ports)
        sys.exit(1)

    for p in ports:
        desc = (p.description or "") + " " + (p.manufacturer or "")
        if any(kw.lower() in desc.lower() for kw in PORT_KEYWORDS):
            print(f"{Fore.GREEN}[AUTO] Detected: {p.device} ({p.description}){Style.RESET_ALL}")
            return p.device

    if not ports:
        print(f"{Fore.RED}[ERROR] No serial ports found.{Style.RESET_ALL}")
        sys.exit(1)

    print(f"{Fore.YELLOW}[WARN] ROBOTIS/OpenCM port not auto-detected.{Style.RESET_ALL}")
    _show_ports(ports)
    choice = input("Enter port name or number: ").strip()
    try:
        idx = int(choice)
        if 0 <= idx < len(ports):
            return ports[idx].device
    except ValueError:
        pass
    return choice


def _show_ports(ports):
    print("\nAvailable ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    print()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def timestamp_str():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def safe_path(path: Path) -> Path:
    """If path already exists, append _2, _3, ... to avoid overwriting."""
    if not path.exists():
        return path
    stem = path.stem
    suffixes = "".join(path.suffixes)
    parent = path.parent
    i = 2
    while True:
        candidate = parent / f"{stem}_{i}{suffixes}"
        if not candidate.exists():
            return candidate
        i += 1


def wait_for_enter(message):
    """Display a prominent message and wait for Enter."""
    print(f"\n{'=' * 60}")
    print(f"{Fore.YELLOW}{message}{Style.RESET_ALL}")
    print(f"{'=' * 60}")
    input(f"{Fore.CYAN}>>> Press Enter to continue...{Style.RESET_ALL}")
    print()


def ask_step_result(label=""):
    """Ask user if the step completed successfully. Returns 'ok', 'retry', or 'quit'."""
    while True:
        print(f"\n{'=' * 60}")
        if label:
            print(f"{Fore.CYAN}  {label} - Check result{Style.RESET_ALL}")
        print(f"    [Enter] Completed successfully")
        print(f"    [r]     Problem occurred → Retry")
        print(f"    [q]     Quit")
        print(f"{'=' * 60}")
        choice = input(f"{Fore.CYAN}>>> Select: {Style.RESET_ALL}").strip().lower()
        if choice == "":
            return "ok"
        elif choice == "r":
            return "retry"
        elif choice == "q":
            return "quit"
        else:
            print(f"{Fore.RED}  Invalid input. Please choose Enter / r / q.{Style.RESET_ALL}")


# ============================================================
# #define Auto-Switch
# ============================================================

def verify_define(condition, target_mode):
    """
    Verify that the #define in the .ino file matches the expected mode.
    Returns True if correct, False otherwise.
    """
    arduino_folder = CONDITION_ARDUINO_MAP.get(condition)
    if not arduino_folder:
        return False
    main_ino = ARDUINO_DIR / arduino_folder / f"{arduino_folder}.ino"
    if not main_ino.exists():
        return False
    content = main_ino.read_text(encoding="utf-8")
    target_define = f"MODE_{target_mode}"
    # Check: target mode should be uncommented
    has_active = bool(re.search(rf"^\s*#define\s+{target_define}\b", content, re.MULTILINE))
    # Check: other modes should be commented out
    other_modes = [m for m in ["MODE_HX711_CALIBRATION", "MODE_FORCE_PROFILING", "MODE_INTEGRATED_CONTROL"] if m != target_define]
    others_inactive = all(
        not re.search(rf"^\s*#define\s+{m}\b", content, re.MULTILINE)
        for m in other_modes
    )
    return has_active and others_inactive


def switch_define(condition, target_mode):
    """
    Switch #define in the main .ino file.
    target_mode: 'FORCE_PROFILING' or 'INTEGRATED_CONTROL'
    """
    arduino_folder = CONDITION_ARDUINO_MAP.get(condition)
    if not arduino_folder:
        print(f"{Fore.RED}[ERROR] Unknown condition: {condition}{Style.RESET_ALL}")
        return False

    main_ino = ARDUINO_DIR / arduino_folder / f"{arduino_folder}.ino"
    if not main_ino.exists():
        print(f"{Fore.RED}[ERROR] File not found: {main_ino}{Style.RESET_ALL}")
        return False

    content = main_ino.read_text(encoding="utf-8")
    original = content

    # All possible modes
    modes = {
        "MODE_HX711_CALIBRATION": False,
        "MODE_FORCE_PROFILING": False,
        "MODE_INTEGRATED_CONTROL": False,
    }
    modes[f"MODE_{target_mode}"] = True

    for mode_name, should_enable in modes.items():
        if should_enable:
            # Uncomment: "// #define MODE_X" → "#define MODE_X"
            content = re.sub(
                rf"^(\s*)//\s*(#define\s+{mode_name}\b.*)$",
                rf"\1\2",
                content,
                flags=re.MULTILINE,
            )
        else:
            # Comment out: "#define MODE_X" → "// #define MODE_X" (skip if already commented)
            content = re.sub(
                rf"^(\s*)(?!//)(\s*#define\s+{mode_name}\b.*)$",
                rf"\1// \2",
                content,
                flags=re.MULTILINE,
            )

    if content == original:
        print(f"{Fore.CYAN}[DEFINE] Already set to MODE_{target_mode}{Style.RESET_ALL}")
        return True

    main_ino.write_text(content, encoding="utf-8")
    print(f"{Fore.GREEN}[DEFINE] Switched to MODE_{target_mode} in {main_ino.name}{Style.RESET_ALL}")

    # Show what changed
    for old_line, new_line in zip(original.splitlines(), content.splitlines()):
        if old_line != new_line:
            print(f"  {Fore.RED}- {old_line.strip()}{Style.RESET_ALL}")
            print(f"  {Fore.GREEN}+ {new_line.strip()}{Style.RESET_ALL}")

    return True


# ============================================================
# SerialMonitor - Core bidirectional serial communication
# ============================================================
class SerialMonitor:
    """Bidirectional serial monitor with line-based callbacks."""

    def __init__(self, port, baud=DEFAULT_BAUD, raw_log_path=None):
        self.port = port
        self.baud = baud
        self.raw_log_path = raw_log_path
        self.ser = None
        self._running = False
        self._read_thread = None
        self._input_thread = None
        self._line_callbacks = []
        self._raw_log_file = None
        self._lock = threading.Lock()

    def add_callback(self, fn):
        self._line_callbacks.append(fn)

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            print(f"{Fore.RED}[ERROR] Cannot open {self.port}: {e}{Style.RESET_ALL}")
            sys.exit(1)

        time.sleep(0.5)

        if self.raw_log_path:
            ensure_dir(self.raw_log_path.parent)
            self._raw_log_file = open(self.raw_log_path, "w", encoding="utf-8")

        self._running = True

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        print(f"{Fore.GREEN}[CONNECTED] {self.port} @ {self.baud} baud{Style.RESET_ALL}")
        print(f"{Fore.CYAN}Type commands and press Enter to send. Ctrl+C to quit.{Style.RESET_ALL}\n")

    def wait(self):
        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self._raw_log_file:
            self._raw_log_file.close()
            self._raw_log_file = None
        print(f"\n{Fore.YELLOW}[DISCONNECTED]{Style.RESET_ALL}")

    def _read_loop(self):
        buffer = b""
        while self._running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue
                buffer += data
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
                    self._handle_line(line)
            except serial.SerialException:
                if self._running:
                    print(f"\n{Fore.RED}[ERROR] Serial connection lost.{Style.RESET_ALL}")
                    self._running = False
                break
            except Exception as e:
                if self._running:
                    print(f"\n{Fore.RED}[ERROR] Read error: {e}{Style.RESET_ALL}")

    def _handle_line(self, line):
        if line.startswith("[LOG]"):
            display = f"{Fore.CYAN}{line}{Style.RESET_ALL}"
        elif line.startswith("[HEADER]"):
            display = f"{Fore.GREEN}{line}{Style.RESET_ALL}"
        elif "===" in line:
            display = f"{Fore.YELLOW}{line}{Style.RESET_ALL}"
        elif line.startswith("[ERR") or line.startswith("[WARN"):
            display = f"{Fore.RED}{line}{Style.RESET_ALL}"
        else:
            display = line
        print(display)

        if self._raw_log_file:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with self._lock:
                self._raw_log_file.write(f"[{ts}] {line}\n")
                self._raw_log_file.flush()

        for cb in self._line_callbacks:
            try:
                cb(line)
            except Exception as e:
                print(f"{Fore.RED}[CB ERROR] {e}{Style.RESET_ALL}")

    def _input_loop(self):
        while self._running:
            try:
                user_input = self._read_input()
                if user_input is None:
                    continue
                if self.ser and self.ser.is_open:
                    self.ser.write((user_input + "\n").encode("utf-8"))
            except EOFError:
                break
            except Exception:
                if not self._running:
                    break

    def _read_input(self):
        if platform.system() == "Windows":
            return self._read_input_windows()
        else:
            return self._read_input_unix()

    def _read_input_windows(self):
        import msvcrt
        chars = []
        while self._running:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == "\r":
                    print()
                    return "".join(chars)
                elif ch == "\x03":
                    self._running = False
                    return None
                elif ch == "\x08":
                    if chars:
                        chars.pop()
                        sys.stdout.write("\b \b")
                        sys.stdout.flush()
                else:
                    chars.append(ch)
                    sys.stdout.write(ch)
                    sys.stdout.flush()
            else:
                time.sleep(0.05)
        return None

    def _read_input_unix(self):
        import select
        while self._running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline()
                if not line:
                    return None
                return line.rstrip("\n")
        return None


# ============================================================
# RecordHandler - Main Experiment Data Recording
# ============================================================
class RecordHandler:
    """Handles [HEADER]/[LOG] parsing and CSV writing."""

    def __init__(self, data_dir: Path):
        self.data_dir = data_dir
        self._csv_file = None
        self._csv_writer = None
        self._point_count = 0
        self._start_time = None
        self._recording = False
        self.completed = False
        ensure_dir(data_dir)

    def on_line(self, line: str):
        if line.startswith("[HEADER]"):
            self._start_recording()
        elif line.startswith("[LOG],") and self._recording:
            self._write_log(line)
        elif "=== 往復完了 ===" in line and self._recording:  # "Round-trip complete" marker from Arduino
            self._finish_recording()

    def _start_recording(self):
        ts = timestamp_str()
        csv_path = safe_path(self.data_dir / f"{ts}.csv")
        if self._csv_file:
            self._csv_file.close()
        self._csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_HEADER.split(","))
        self._point_count = 0
        self._start_time = time.time()
        self._recording = True
        print(f"\n{Fore.GREEN}>>> CSV recording started: {csv_path.name}{Style.RESET_ALL}")

    def _write_log(self, line: str):
        data = line[len("[LOG],"):]
        fields = [f.strip() for f in data.split(",")]
        if len(fields) >= 6:
            self._csv_writer.writerow(fields)
            self._csv_file.flush()
            self._point_count += 1

    def _finish_recording(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        elapsed = time.time() - self._start_time if self._start_time else 0
        self._recording = False
        self.completed = True
        print(f"\n{Fore.GREEN}>>> Recording complete: {self._point_count} data points, "
              f"{elapsed:.1f}s elapsed{Style.RESET_ALL}")

    def close(self):
        if self._csv_file:
            self._csv_file.close()


# ============================================================
# CalibrateHandler - Automatic Calibration Processing
# ============================================================
class CalibrateHandler:
    """Parses calibration output and patches integrated_control.ino."""

    def __init__(self, condition: str, subject: str, dry_run: bool = False):
        self.condition = condition
        self.subject = subject
        self.dry_run = dry_run

        self.forward_data = []
        self.backward_data = []
        self.excel_sections = []

        self._state = "idle"
        self._excel_current_section = None
        self._excel_current_direction = None
        self._excel_rows = []

        self._trajectory_re = re.compile(r"\{([\d.eE+-]+),([\d.eE+-]+)\}")
        self._excel_row_re = re.compile(r"^(\d+),\s*([\d.eE+-]+),\s*([\d.eE+-]+)$")

        self.cal_data_dir = DATA_DIR / "calibration" / condition
        ensure_dir(self.cal_data_dir)

        self.patch_success = False

    def on_line(self, line: str):
        stripped = line.strip()

        # --- Trajectory data parsing ---
        # NOTE: Japanese strings below are protocol markers sent by Arduino firmware
        if "往路データ (Forward):" in stripped:  # "Forward path data"
            self._state = "forward"
            self.forward_data = []
            return
        if "復路データ (Backward):" in stripped:  # "Backward path data"
            self._state = "backward"
            self.backward_data = []
            return

        if self._state in ("forward", "backward"):
            m = self._trajectory_re.search(stripped)
            if m:
                point = (float(m.group(1)), float(m.group(2)))
                if self._state == "forward":
                    self.forward_data.append(point)
                else:
                    self.backward_data.append(point)
                return
            if stripped and not stripped.startswith("{") and self._state != "idle":
                if "---" in stripped or "===" in stripped or stripped.startswith("復路") or stripped.startswith("往路"):
                    pass
                else:
                    self._state = "idle"

        # --- Excel analysis data parsing ---
        if "=== FORWARD DATA START ===" in stripped:
            self._flush_excel_section()
            self._excel_current_direction = "forward"
            return
        if "=== BACKWARD DATA START ===" in stripped:
            self._flush_excel_section()
            self._excel_current_direction = "backward"
            return
        if "=== データ出力完了 ===" in stripped:  # "Data output complete" marker from Arduino
            self._flush_excel_section()
            self._excel_current_direction = None
            self._on_data_complete()
            return

        if self._excel_current_direction:
            if "静的計測データ" in stripped:  # "Static measurement data" marker from Arduino
                self._flush_excel_section()
                self._excel_current_section = "static_measurement"
                self._excel_rows = []
                return
            dyn_match = re.search(r"動的(\d+)回目データ", stripped)  # "Dynamic Nth trial data" marker from Arduino
            if dyn_match:
                self._flush_excel_section()
                self._excel_current_section = f"dynamic_trial_{dyn_match.group(1)}"
                self._excel_rows = []
                return
            if stripped == "point, angle, force":
                return
            if self._excel_current_section:
                m = self._excel_row_re.match(stripped)
                if m:
                    self._excel_rows.append((int(m.group(1)), float(m.group(2)), float(m.group(3))))

    def _flush_excel_section(self):
        if self._excel_current_section and self._excel_rows:
            self.excel_sections.append({
                "direction": self._excel_current_direction,
                "section_name": self._excel_current_section,
                "rows": list(self._excel_rows),
            })
        self._excel_rows = []

    def _on_data_complete(self):
        print(f"\n{Fore.GREEN}{'=' * 50}")
        print("=== Calibration Data Received ===")
        print(f"{'=' * 50}{Style.RESET_ALL}")

        print(f"\n{Fore.CYAN}Forward trajectory: {len(self.forward_data)} points{Style.RESET_ALL}")
        for py, ma in self.forward_data:
            print(f"  {{{py:.4f},{ma:.2f}}}")

        print(f"\n{Fore.CYAN}Backward trajectory: {len(self.backward_data)} points{Style.RESET_ALL}")
        for py, ma in self.backward_data:
            print(f"  {{{py:.4f},{ma:.2f}}}")

        self._save_excel_csv()

        if self.forward_data and self.backward_data:
            self._patch_ino()
        else:
            print(f"\n{Fore.RED}[ERROR] No trajectory data parsed. Skipping .ino patch.{Style.RESET_ALL}")

    def _save_excel_csv(self):
        if not self.excel_sections:
            return
        ts = timestamp_str()
        csv_path = safe_path(self.cal_data_dir / f"{ts}_learning.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            for section in self.excel_sections:
                writer.writerow([f"{section['direction']} - {section['section_name']}"])
                writer.writerow(["point", "angle", "force"])
                for row in section["rows"]:
                    writer.writerow(row)
                writer.writerow([])
        print(f"\n{Fore.GREEN}>>> Calibration CSV saved: {csv_path}{Style.RESET_ALL}")

    def _patch_ino(self):
        arduino_folder = CONDITION_ARDUINO_MAP.get(self.condition)
        if not arduino_folder:
            print(f"{Fore.RED}[ERROR] No Arduino folder for condition '{self.condition}'.{Style.RESET_ALL}")
            return

        ino_path = ARDUINO_DIR / arduino_folder / "integrated_control.ino"
        if not ino_path.exists():
            print(f"{Fore.RED}[ERROR] File not found: {ino_path}{Style.RESET_ALL}")
            return

        original = ino_path.read_text(encoding="utf-8")

        fwd_points = self._format_trajectory_array(self.forward_data)
        bwd_points = self._format_trajectory_array(self.backward_data)

        new_content = original
        new_content = self._replace_trajectory_block(
            new_content, "TRAJECTORY_POINTS_FORWARD", "g_trajectory_forward",
            len(self.forward_data), fwd_points)
        new_content = self._replace_trajectory_block(
            new_content, "TRAJECTORY_POINTS_BACKWARD", "g_trajectory_backward",
            len(self.backward_data), bwd_points)

        if new_content == original:
            print(f"\n{Fore.YELLOW}[INFO] No changes needed - data is identical.{Style.RESET_ALL}")
            self.patch_success = True
            return

        print(f"\n{Fore.CYAN}--- Patch Preview for {ino_path.name} ---{Style.RESET_ALL}")
        self._show_diff(original, new_content)

        if self.dry_run:
            print(f"\n{Fore.YELLOW}[DRY-RUN] No changes written.{Style.RESET_ALL}")
            return

        confirm = input(f"\n{Fore.YELLOW}Apply patch? (y/N): {Style.RESET_ALL}").strip().lower()
        if confirm != "y":
            print("Patch cancelled.")
            return

        ino_path.write_text(new_content, encoding="utf-8")
        print(f"{Fore.GREEN}>>> Patched: {ino_path.name}{Style.RESET_ALL}")
        self.patch_success = True

        # Backup AFTER patch (so backup contains the current calibration data)
        ts = timestamp_str()
        bak_name = f"integrated_control_{self.subject}_{self.condition}_{ts[:8]}.ino.bak"
        bak_path = ino_path.parent / bak_name
        shutil.copy2(ino_path, bak_path)
        print(f"{Fore.GREEN}>>> Backup (patched): {bak_path.name}{Style.RESET_ALL}")

    def _format_trajectory_array(self, data):
        lines = []
        for py, ma in data:
            lines.append(f"  {{{py:.4f},{ma:.2f}}},")
        return "\n".join(lines)

    def _replace_trajectory_block(self, content, const_name, array_name, count, array_body):
        content = re.sub(
            rf"(const\s+int\s+{const_name}\s*=\s*)\d+(\s*;)",
            rf"\g<1>{count}\2", content)
        pattern = (
            rf"(const\s+TrajectoryPoint\s+{array_name}\[{const_name}\]\s*=\s*\{{\s*\n)"
            rf"(.*?)"
            rf"(\}};)")
        replacement = rf"\g<1>{array_body}\n\3"
        content = re.sub(pattern, replacement, content, flags=re.DOTALL)
        return content

    def _show_diff(self, old, new):
        old_lines = old.splitlines()
        new_lines = new.splitlines()
        diff = difflib.unified_diff(old_lines, new_lines, lineterm="", n=2)
        for line in diff:
            if line.startswith("+++") or line.startswith("---"):
                print(f"{Fore.CYAN}{line}{Style.RESET_ALL}")
            elif line.startswith("+"):
                print(f"{Fore.GREEN}{line}{Style.RESET_ALL}")
            elif line.startswith("-"):
                print(f"{Fore.RED}{line}{Style.RESET_ALL}")
            elif line.startswith("@@"):
                print(f"{Fore.YELLOW}{line}{Style.RESET_ALL}")


# ============================================================
# Mode: ports
# ============================================================
def cmd_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print(f"\n{'Device':<20} {'Description':<40} {'Manufacturer'}")
    print("-" * 80)
    for p in ports:
        marker = ""
        desc = (p.description or "") + " " + (p.manufacturer or "")
        if any(kw.lower() in desc.lower() for kw in PORT_KEYWORDS):
            marker = f" {Fore.GREEN}<< ROBOTIS{Style.RESET_ALL}"
        print(f"{p.device:<20} {p.description or '':<40} {p.manufacturer or ''}{marker}")
    print()


# ============================================================
# Mode: record
# ============================================================
def cmd_record(args):
    data_dir = DATA_DIR / args.condition / args.subject
    ensure_dir(data_dir)

    ts = timestamp_str()
    raw_log_path = safe_path(data_dir / f"{ts}_raw.log")

    port = find_port(args.port)
    handler = RecordHandler(data_dir)

    print(f"\n{Fore.CYAN}=== MOBITOUCH Record Mode ==={Style.RESET_ALL}")
    print(f"  Condition: {args.condition}")
    print(f"  Subject:   {args.subject}")
    print(f"  Data dir:  {data_dir}")
    print(f"  Raw log:   {raw_log_path.name}")
    print()

    while True:
        monitor = SerialMonitor(port, args.baud, raw_log_path=raw_log_path)
        monitor.add_callback(handler.on_line)
        monitor.start()
        monitor.wait()
        handler.close()

        result = ask_step_result(f"Record ({args.condition})")
        if result == "ok":
            break
        elif result == "retry":
            print(f"{Fore.YELLOW}  Retrying...{Style.RESET_ALL}")
            ts = timestamp_str()
            raw_log_path = safe_path(data_dir / f"{ts}_raw.log")
            handler = RecordHandler(data_dir)
            port = find_port(args.port)
            continue
        else:
            sys.exit(0)


# ============================================================
# Mode: calibrate
# ============================================================
def cmd_calibrate(args):
    if args.condition not in CONDITIONS_CALIBRATE:
        print(f"{Fore.RED}[ERROR] Calibration only supported for: {CONDITIONS_CALIBRATE}{Style.RESET_ALL}")
        sys.exit(1)

    # Step 1: Switch #define to FORCE_PROFILING
    print(f"\n{Fore.CYAN}=== MOBITOUCH Calibrate Mode ==={Style.RESET_ALL}")
    print(f"  Condition: {args.condition}")
    print(f"  Subject:   {args.subject}")
    print()

    switch_define(args.condition, "FORCE_PROFILING")

    wait_for_enter(
        f"[Step 1] Calibration Preparation\n"
        f"  1. Reset microcontroller (hold white button for 2 sec)\n"
        f"  2. Upload {CONDITION_ARDUINO_MAP[args.condition]}.ino from Arduino IDE\n"
        f"     * Keep load cell unloaded during upload\n"
        f"  3. Adjust stroking module height (* with external power OFF)\n"
        f"  4. Turn stabilization power ON (7.40V)\n"
        f"  5. Place robot at start position"
    )

    # Step 2: Connect serial and run calibration
    cal_data_dir = DATA_DIR / "calibration" / args.condition
    ensure_dir(cal_data_dir)
    ts = timestamp_str()
    raw_log_path = safe_path(cal_data_dir / f"{ts}_raw.log")

    port = find_port(args.port)
    handler = CalibrateHandler(args.condition, args.subject, dry_run=args.dry_run)

    print(f"  Target: Arduino/{CONDITION_ARDUINO_MAP[args.condition]}/integrated_control.ino")
    print(f"  Raw log: {raw_log_path.name}")
    print()
    print(f"{Fore.YELLOW}  Type 'p' in serial monitor to start calibration.{Style.RESET_ALL}")
    print(f"{Fore.YELLOW}  After calibration completes, .ino patch will be applied automatically.{Style.RESET_ALL}")
    print()

    while True:
        monitor = SerialMonitor(port, DEFAULT_BAUD, raw_log_path=raw_log_path)
        monitor.add_callback(handler.on_line)
        monitor.start()
        monitor.wait()

        result = ask_step_result(f"Calibration ({args.condition})")
        if result == "ok":
            break
        elif result == "retry":
            print(f"{Fore.YELLOW}  Retrying...{Style.RESET_ALL}")
            ts = timestamp_str()
            raw_log_path = safe_path(cal_data_dir / f"{ts}_raw.log")
            handler = CalibrateHandler(args.condition, args.subject, dry_run=args.dry_run)
            port = find_port(args.port)
            continue
        else:
            sys.exit(0)


# ============================================================
# Mode: run (full workflow: calibrate → record)
# ============================================================
def cmd_run(args):
    """Full automated workflow for parallel/orthogonal conditions."""
    condition = args.condition
    subject = args.subject
    port_name = args.port

    if condition not in CONDITIONS_CALIBRATE:
        print(f"{Fore.RED}[ERROR] 'run' mode is for parallel/orthogonal only: {CONDITIONS_CALIBRATE}")
        print(f"For PID conditions, use 'record' mode directly.{Style.RESET_ALL}")
        sys.exit(1)

    arduino_folder = CONDITION_ARDUINO_MAP[condition]

    print(f"\n{'=' * 60}")
    print(f"{Fore.CYAN}  MOBITOUCH Full Experiment Flow{Style.RESET_ALL}")
    print(f"  Condition: {condition}")
    print(f"  Subject:   {subject}")
    print(f"{'=' * 60}\n")

    # ========================================
    # PHASE 1: Calibration
    # ========================================
    print(f"{Fore.YELLOW}{'=' * 60}")
    print(f"  PHASE 1: Calibration (Force Profiling)")
    print(f"{'=' * 60}{Style.RESET_ALL}\n")

    # 1a: Switch #define
    switch_define(condition, "FORCE_PROFILING")

    # 1b: Wait for user to upload & prepare
    wait_for_enter(
        f"[PHASE 1] Calibration Preparation\n"
        f"  1. Reset microcontroller (hold white button for 2 sec)\n"
        f"  2. Upload {arduino_folder}.ino from Arduino IDE\n"
        f"     * Keep load cell unloaded during upload\n"
        f"  3. Adjust stroking module height (* with external power OFF)\n"
        f"  4. Turn stabilization power ON (7.40V)\n"
        f"  5. Place robot at start position\n"
        f"  Press Enter when ready"
    )

    # 1c: Serial monitor for calibration
    cal_data_dir = DATA_DIR / "calibration" / condition
    ensure_dir(cal_data_dir)
    ts = timestamp_str()
    raw_log_path = safe_path(cal_data_dir / f"{ts}_raw.log")

    port = find_port(port_name)
    cal_handler = CalibrateHandler(condition, subject)
    monitor = SerialMonitor(port, DEFAULT_BAUD, raw_log_path=raw_log_path)
    monitor.add_callback(cal_handler.on_line)

    print(f"{Fore.YELLOW}  Type 'p' to start calibration.{Style.RESET_ALL}")
    print(f"{Fore.YELLOW}  Dynamic learning: return robot to home position before each trial, then press Enter{Style.RESET_ALL}")
    print(f"{Fore.YELLOW}  After completion, .ino will be auto-patched → Ctrl+C to exit{Style.RESET_ALL}\n")

    monitor.start()
    monitor.wait()

    if not cal_handler.patch_success:
        print(f"\n{Fore.RED}[ERROR] Calibration patch failed. Aborting.{Style.RESET_ALL}")
        sys.exit(1)

    # ========================================
    # PHASE 2: Main Experiment
    # ========================================
    print(f"\n{Fore.YELLOW}{'=' * 60}")
    print(f"  PHASE 2: Main Experiment (Integrated Control)")
    print(f"{'=' * 60}{Style.RESET_ALL}\n")

    # 2a: Switch #define
    switch_define(condition, "INTEGRATED_CONTROL")

    # 2b: Wait for user to upload & prepare
    wait_for_enter(
        f"[PHASE 2] Main Experiment Preparation\n"
        f"  1. Turn stabilization power OFF\n"
        f"  2. Reset microcontroller (hold white button for 2 sec)\n"
        f"  3. Upload {arduino_folder}.ino from Arduino IDE\n"
        f"     * Keep load cell unloaded during upload\n"
        f"  4. Adjust stroking module height (* with external power OFF)\n"
        f"  5. Turn stabilization power ON (7.40V)\n"
        f"  6. Place robot at start position\n"
        f"  Press Enter when ready"
    )

    # 2c: Serial monitor for recording
    data_dir = DATA_DIR / condition / subject
    ensure_dir(data_dir)
    ts = timestamp_str()
    raw_log_path = safe_path(data_dir / f"{ts}_raw.log")

    port = find_port(port_name)
    rec_handler = RecordHandler(data_dir)
    monitor = SerialMonitor(port, DEFAULT_BAUD, raw_log_path=raw_log_path)
    monitor.add_callback(rec_handler.on_line)

    print(f"  Data dir: {data_dir}")
    print(f"{Fore.YELLOW}  1. Type 'all start'{Style.RESET_ALL}")
    print(f"{Fore.YELLOW}  2. After positioning subject's arm, press space + Enter → simultaneously press EDA marker button{Style.RESET_ALL}")
    print(f"{Fore.YELLOW}  3. After 10 round-trips, CSV auto-saves → turn external power OFF → Ctrl+C{Style.RESET_ALL}\n")

    monitor.start()
    monitor.wait()
    rec_handler.close()

    # ========================================
    # PHASE 3: Cleanup
    # ========================================
    # Switch back to FORCE_PROFILING for next subject
    switch_define(condition, "FORCE_PROFILING")

    print(f"\n{Fore.GREEN}{'=' * 60}")
    print(f"  Experiment Complete!")
    print(f"  Condition: {condition}")
    print(f"  Subject:   {subject}")
    print(f"  #define: Reset to MODE_FORCE_PROFILING (ready for next subject)")
    print(f"{'=' * 60}{Style.RESET_ALL}\n")


# ============================================================
# Experiment Order Generation
# ============================================================
ORDER_CSV_PATH = DATA_DIR / "experiment_order.csv"

def generate_order(n_subjects=20, seed=42):
    """
    Generate counterbalanced experiment order CSV.

    Rules:
    - Odd subjects (S01,S03,...): parallel block first
    - Even subjects (S02,S04,...): orthogonal block first
    - Each block: calibration → {condition, condition_PID} in random order
    - Within-block PID vs non-PID order is randomized per subject
    """
    import random
    rng = random.Random(seed)

    ensure_dir(DATA_DIR)

    rows = []
    for i in range(1, n_subjects + 1):
        subject = f"S{i:02d}"

        # Block order: odd=parallel first, even=orthogonal first
        if i % 2 == 1:
            blocks = ["parallel", "orthogonal"]
        else:
            blocks = ["orthogonal", "parallel"]

        step = 1
        for direction in blocks:
            # Calibration always first in block
            rows.append([subject, step, "calibrate", direction])
            step += 1

            # Random order of PID vs non-PID within block
            pair = [direction, f"{direction}_PID"]
            rng.shuffle(pair)

            for cond in pair:
                rows.append([subject, step, "record", cond])
                step += 1

    with open(ORDER_CSV_PATH, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["subject", "step", "action", "condition"])
        writer.writerows(rows)

    print(f"{Fore.GREEN}>>> Experiment order saved: {ORDER_CSV_PATH}{Style.RESET_ALL}")
    print(f"    Subjects: S01-S{n_subjects:02d}, Seed: {seed}\n")

    # Preview
    current_subject = None
    for row in rows:
        subj, step, action, cond = row
        if subj != current_subject:
            current_subject = subj
            print(f"  {Fore.CYAN}{subj}:{Style.RESET_ALL}")
        marker = "CAL" if action == "calibrate" else "REC"
        print(f"    {step}. [{marker}] {cond}")
    print()


def load_order(subject):
    """Load experiment steps for a subject from the order CSV."""
    if not ORDER_CSV_PATH.exists():
        print(f"{Fore.RED}[ERROR] {ORDER_CSV_PATH} not found.")
        print(f"Run 'generate-order' first.{Style.RESET_ALL}")
        sys.exit(1)

    steps = []
    with open(ORDER_CSV_PATH, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["subject"] == subject:
                steps.append(row)

    if not steps:
        print(f"{Fore.RED}[ERROR] Subject '{subject}' not found in {ORDER_CSV_PATH.name}{Style.RESET_ALL}")
        sys.exit(1)

    return steps


# ============================================================
# Mode: session (full experiment for one subject)
# ============================================================
def cmd_session(args):
    """Run all conditions for a subject based on experiment_order.csv."""
    subject = args.subject
    port_name = args.port

    steps = load_order(subject)

    print(f"\n{'=' * 60}")
    print(f"{Fore.CYAN}  MOBITOUCH Full Session for {subject}{Style.RESET_ALL}")
    print(f"{'=' * 60}")
    print(f"\n  Experiment order:")
    for s in steps:
        marker = "CAL" if s["action"] == "calibrate" else "REC"
        print(f"    Step {s['step']}. [{marker}] {s['condition']}")
    print()

    # Pre-experiment: 3D arm scan
    wait_for_enter(
        f"[Pre-experiment] 3D Arm Scan (Polycam)\n"
        f"  1. Place subject's forearm on the table\n"
        f"  2. Scan with Polycam LiDAR mode at 30-50cm distance\n"
        f"  3. Export as .obj file → save to {subject} folder\n"
        f"  Press Enter after scan is complete"
    )

    total_steps = len(steps)
    prev_direction = None

    i = 0
    while i < total_steps:
        step = steps[i]
        action = step["action"]
        condition = step["condition"]
        step_num = step["step"]

        # Detect direction change (parallel ↔ orthogonal)
        current_direction = "parallel" if "parallel" in condition else "orthogonal"
        if prev_direction is not None and current_direction != prev_direction:
            wait_for_enter(
                f"Direction change: {prev_direction} → {current_direction}\n"
                f"  Change the orientation of the stroking module.\n"
                f"  1. Loosen mounting bolts and rotate module\n"
                f"  2. Re-tighten bolts\n"
                f"  Press Enter after module change is complete"
            )
        prev_direction = current_direction

        print(f"\n{'=' * 60}")
        print(f"{Fore.YELLOW}  Step {step_num}/{total_steps}: "
              f"{'Calibration' if action == 'calibrate' else 'Main Experiment'} "
              f"- {condition}{Style.RESET_ALL}")
        print(f"{'=' * 60}")

        arduino_folder = CONDITION_ARDUINO_MAP[condition]

        if action == "calibrate":
            # --- Calibration ---
            switch_define(condition, "FORCE_PROFILING")
            if not verify_define(condition, "FORCE_PROFILING"):
                print(f"{Fore.RED}[ERROR] #define verification failed! MODE_FORCE_PROFILING is not active.{Style.RESET_ALL}")
                print(f"{Fore.RED}  Please check the .ino file manually.{Style.RESET_ALL}")
                confirm = input("Continue anyway? (y/N): ").strip().lower()
                if confirm != "y":
                    sys.exit(1)

            wait_for_enter(
                f"[Step {step_num}] Calibration Preparation ({condition})\n"
                f"  1. Reset microcontroller (hold white button for 2 sec)\n"
                f"  2. Upload {arduino_folder}.ino from Arduino IDE\n"
                f"     * Keep load cell unloaded during upload\n"
                f"  3. Adjust stroking module height (* with external power OFF)\n"
                f"  4. Turn stabilization power ON (7.40V)\n"
                f"  5. Place robot at start position"
            )

            cal_data_dir = DATA_DIR / "calibration" / condition
            ensure_dir(cal_data_dir)
            ts = timestamp_str()
            raw_log_path = safe_path(cal_data_dir / f"{ts}_raw.log")

            port = find_port(port_name)
            cal_handler = CalibrateHandler(condition, subject)
            monitor = SerialMonitor(port, DEFAULT_BAUD, raw_log_path=raw_log_path)
            monitor.add_callback(cal_handler.on_line)

            print(f"{Fore.YELLOW}  Type 'p' → start calibration{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}  After completion, .ino auto-patch → Ctrl+C for next step{Style.RESET_ALL}\n")

            monitor.start()
            monitor.wait()

            if not cal_handler.patch_success:
                print(f"\n{Fore.RED}[ERROR] Calibration patch failed.{Style.RESET_ALL}")
                confirm = input("Continue anyway? (y/N): ").strip().lower()
                if confirm != "y":
                    sys.exit(1)

            # After calibration, power off
            print(f"\n{Fore.YELLOW}  Calibration complete. Turn stabilization power OFF → remove robot{Style.RESET_ALL}")

        else:
            # --- Record (main experiment) ---
            # Determine which Arduino folder to use for #define
            base_condition = condition.replace("_PID", "")
            arduino_folder_for_define = CONDITION_ARDUINO_MAP.get(condition, CONDITION_ARDUINO_MAP.get(base_condition))

            switch_define(condition, "INTEGRATED_CONTROL")
            if not verify_define(condition, "INTEGRATED_CONTROL"):
                print(f"{Fore.RED}[ERROR] #define verification failed! MODE_INTEGRATED_CONTROL is not active.{Style.RESET_ALL}")
                print(f"{Fore.RED}  Please check the .ino file manually.{Style.RESET_ALL}")
                confirm = input("Continue anyway? (y/N): ").strip().lower()
                if confirm != "y":
                    sys.exit(1)

            wait_for_enter(
                f"[Step {step_num}] Main Experiment Preparation ({condition})\n"
                f"  1. Reset microcontroller (hold white button for 2 sec)\n"
                f"  2. Upload {CONDITION_ARDUINO_MAP[condition]}.ino from Arduino IDE\n"
                f"     * Keep load cell unloaded during upload\n"
                f"  3. Adjust stroking module height (* with external power OFF)\n"
                f"  4. Turn stabilization power ON (7.40V)\n"
                f"  5. Place robot at start position"
            )

            data_dir = DATA_DIR / condition / subject
            ensure_dir(data_dir)
            ts = timestamp_str()
            raw_log_path = safe_path(data_dir / f"{ts}_raw.log")

            port = find_port(port_name)
            rec_handler = RecordHandler(data_dir)
            monitor = SerialMonitor(port, DEFAULT_BAUD, raw_log_path=raw_log_path)
            monitor.add_callback(rec_handler.on_line)

            print(f"  Data dir: {data_dir}")
            print(f"{Fore.YELLOW}  1. Type 'all start'{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}  2. After positioning subject's arm, press space + Enter → simultaneously press EDA marker button{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}  3. After completion, turn stabilization power OFF → Ctrl+C for next step{Style.RESET_ALL}\n")

            monitor.start()
            monitor.wait()
            rec_handler.close()

        # Step result check
        result = ask_step_result(f"Step {step_num} ({condition})")
        if result == "retry":
            print(f"{Fore.YELLOW}  Retrying the same step...{Style.RESET_ALL}")
            prev_direction = None
            continue  # Don't increment i, retry same step
        elif result == "quit":
            print(f"\n{Fore.YELLOW}  Session aborted.{Style.RESET_ALL}")
            sys.exit(0)

        # Progress
        i += 1
        remaining = total_steps - i
        if remaining > 0:
            print(f"\n{Fore.GREEN}  ✓ Step {step_num} complete. "
                  f"Remaining: {remaining} step(s){Style.RESET_ALL}")
        else:
            print(f"\n{Fore.GREEN}{'=' * 60}")
            print(f"  Session Complete! ({subject})")
            print(f"  All {total_steps} steps finished.")
            print(f"{'=' * 60}{Style.RESET_ALL}\n")


# ============================================================
# CLI Entry Point
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="MOBITOUCH Serial Logger - Experiment data recorder & calibration tool"
    )
    subparsers = parser.add_subparsers(dest="mode")

    # session (full experiment for one subject)
    ses = subparsers.add_parser("session",
        help="Run all conditions for a subject (reads experiment_order.csv)")
    ses.add_argument("--subject", required=True, help="Subject ID (e.g., S01)")
    ses.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")

    # generate-order
    gen = subparsers.add_parser("generate-order",
        help="Generate counterbalanced experiment order CSV")
    gen.add_argument("--subjects", default=20, type=int, help="Number of subjects (default: 20)")
    gen.add_argument("--seed", default=42, type=int, help="Random seed (default: 42)")

    # run (single condition: calibrate → record)
    run_p = subparsers.add_parser("run",
        help="Single condition flow: calibrate → record (parallel/orthogonal)")
    run_p.add_argument("--condition", required=True, choices=CONDITIONS_CALIBRATE,
                       help="Experiment condition (parallel or orthogonal)")
    run_p.add_argument("--subject", required=True, help="Subject ID (e.g., S01)")
    run_p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")

    # record
    rec = subparsers.add_parser("record", help="Record experiment serial data to CSV")
    rec.add_argument("--condition", required=True, choices=CONDITIONS_ALL,
                     help="Experiment condition")
    rec.add_argument("--subject", required=True, help="Subject ID (e.g., S01)")
    rec.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    rec.add_argument("--baud", default=DEFAULT_BAUD, type=int, help="Baud rate")

    # calibrate
    cal = subparsers.add_parser("calibrate",
        help="Calibration only: data capture + .ino auto-patch")
    cal.add_argument("--condition", required=True, choices=CONDITIONS_CALIBRATE,
                     help="Experiment condition")
    cal.add_argument("--subject", required=True, help="Subject ID (e.g., S01)")
    cal.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    cal.add_argument("--dry-run", action="store_true",
                     help="Show patch preview without modifying files")

    # ports
    subparsers.add_parser("ports", help="List available serial ports")

    args = parser.parse_args()

    if args.mode == "session":
        cmd_session(args)
    elif args.mode == "generate-order":
        generate_order(n_subjects=args.subjects, seed=args.seed)
    elif args.mode == "run":
        cmd_run(args)
    elif args.mode == "record":
        cmd_record(args)
    elif args.mode == "calibrate":
        cmd_calibrate(args)
    elif args.mode == "ports":
        cmd_ports()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
