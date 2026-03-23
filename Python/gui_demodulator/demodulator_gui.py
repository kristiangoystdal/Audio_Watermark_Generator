#!/usr/bin/env python3
import math
import os
import re
import threading
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox
from tkinter import ttk
from pathlib import Path

import soundfile as sf

from demodulator import decode_fsk
from reed_solomon import NSYM as DEFAULT_ECC_NSYM

# ------------------------------
# Frequency helpers constants
# ------------------------------
FS_HZ = 960000
MIN_BIT_US = 3000
FREQ_MIN = 2000
FREQ_MAX = 24000
BIT_SAMPLE_TOLERANCE_PERCENT = 1
SYNC_FILE_SUFFIX = "_synced"
LABEL_TIMESTAMP_RE = re.compile(
    r"Time:\s*"
    r"(?P<hour>\d{2}):(?P<minute>\d{2}):(?P<second>\d{2})"
    r"\s+on\s+"
    r"(?P<weekday>[A-Za-z]+)\s+"
    r"(?P<day>\d{2})/(?P<month>\d{2})/(?P<year>\d{4})"
)


def derive_txt_path(wav_path: str) -> str:
    base, _ = os.path.splitext(wav_path)
    return base + ".txt"


def derive_synced_wav_path(wav_path: str) -> str:
    base, ext = os.path.splitext(wav_path)
    return base + SYNC_FILE_SUFFIX + ext


def derive_synced_txt_path(wav_path: str) -> str:
    base, _ = os.path.splitext(wav_path)
    return base + SYNC_FILE_SUFFIX + ".txt"


@dataclass(frozen=True)
class LabelEntry:
    start_time: float
    end_time: float
    label_text: str
    message_timestamp: datetime | None


@dataclass(frozen=True)
class SyncResult:
    outputs: tuple[str, ...]
    message: str
    shared_timestamp: datetime | None = None


def extract_message_timestamp(label_text: str) -> datetime | None:
    match = LABEL_TIMESTAMP_RE.search(label_text)
    if match is None:
        return None

    try:
        return datetime(
            year=int(match.group("year")),
            month=int(match.group("month")),
            day=int(match.group("day")),
            hour=int(match.group("hour")),
            minute=int(match.group("minute")),
            second=int(match.group("second")),
        )
    except ValueError:
        return None


def parse_label_entries(txt_path: str) -> list[LabelEntry]:
    entries: list[LabelEntry] = []
    with open(txt_path, "r", encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue

            parts = line.split("\t", 2)
            if len(parts) != 3:
                raise ValueError(
                    f"Invalid label line in {Path(txt_path).name} at line {line_no}"
                )

            try:
                start_time = float(parts[0])
                end_time = float(parts[1])
            except ValueError as exc:
                raise ValueError(
                    f"Invalid time range in {Path(txt_path).name} at line {line_no}"
                ) from exc

            label_text = parts[2]
            entries.append(
                LabelEntry(
                    start_time=start_time,
                    end_time=end_time,
                    label_text=label_text,
                    message_timestamp=extract_message_timestamp(label_text),
                )
            )
    return entries


def find_shared_message_timestamp(
    entries_by_file: dict[str, list[LabelEntry]],
) -> datetime | None:
    timestamp_sets = []
    for entries in entries_by_file.values():
        timestamps = {
            entry.message_timestamp
            for entry in entries
            if entry.message_timestamp is not None
        }
        if not timestamps:
            return None
        timestamp_sets.append(timestamps)

    if not timestamp_sets:
        return None

    shared = set.intersection(*timestamp_sets)
    if not shared:
        return None
    return min(shared)


def write_synced_label_file(
    output_txt_path: str,
    entries: list[LabelEntry],
    crop_start_time: float,
    crop_duration: float,
):
    crop_end_time = crop_start_time + crop_duration
    with open(output_txt_path, "w", encoding="utf-8") as handle:
        for entry in entries:
            if entry.start_time + 1e-9 < crop_start_time:
                continue
            if entry.end_time - 1e-9 > crop_end_time:
                continue

            synced_start = entry.start_time - crop_start_time
            synced_end = entry.end_time - crop_start_time
            handle.write(
                f"{synced_start:.6f}\t{synced_end:.6f}\t{entry.label_text}\n"
            )


def synchronize_audio_files(wav_paths: list[str]) -> SyncResult:
    if len(wav_paths) < 2:
        return SyncResult(
            outputs=(),
            message="Automatic synchronization needs at least two audio files.",
        )

    entries_by_file: dict[str, list[LabelEntry]] = {}
    for wav_path in wav_paths:
        txt_path = derive_txt_path(wav_path)
        if not os.path.exists(txt_path):
            raise FileNotFoundError(f"Missing label file: {Path(txt_path).name}")
        entries_by_file[wav_path] = parse_label_entries(txt_path)

    shared_timestamp = find_shared_message_timestamp(entries_by_file)
    if shared_timestamp is None:
        return SyncResult(
            outputs=(),
            message=(
                "No shared message timestamp was found across all selected files. "
                "No synchronized copies were created."
            ),
        )

    crop_plan: dict[str, dict[str, object]] = {}
    sample_rates = set()
    anchor_samples: dict[str, int] = {}
    anchor_times: dict[str, float] = {}
    min_available_frames: int | None = None
    min_available_duration: float | None = None

    for wav_path, entries in entries_by_file.items():
        matching_entries = [
            entry for entry in entries if entry.message_timestamp == shared_timestamp
        ]
        if not matching_entries:
            raise ValueError(f"Shared timestamp lookup failed for {Path(wav_path).name}")

        anchor = min(matching_entries, key=lambda entry: entry.start_time)
        info = sf.info(wav_path)
        sample_rates.add(info.samplerate)

        crop_plan[wav_path] = {"info": info}
        anchor_sample = int(round(anchor.start_time * info.samplerate))
        anchor_sample = max(0, min(anchor_sample, info.frames))
        anchor_samples[wav_path] = anchor_sample
        anchor_times[wav_path] = (
            anchor_sample / info.samplerate if info.samplerate else 0.0
        )

    if len(sample_rates) == 1:
        target_anchor_sample = min(anchor_samples.values())
        for wav_path in wav_paths:
            info = crop_plan[wav_path]["info"]
            start_sample = anchor_samples[wav_path] - target_anchor_sample
            available_frames = info.frames - start_sample
            crop_plan[wav_path]["start_sample"] = start_sample
            crop_plan[wav_path]["crop_start_time"] = (
                start_sample / info.samplerate if info.samplerate else 0.0
            )
            crop_plan[wav_path]["available_frames"] = available_frames

            if min_available_frames is None or available_frames < min_available_frames:
                min_available_frames = available_frames
        if min_available_frames is None or min_available_frames <= 0:
            raise ValueError("Could not determine a valid synchronized crop window.")
        min_available_duration = min_available_frames / next(iter(sample_rates))
    else:
        target_anchor_time = min(anchor_times.values())
        for wav_path in wav_paths:
            info = crop_plan[wav_path]["info"]
            crop_start_time = max(0.0, anchor_times[wav_path] - target_anchor_time)
            start_sample = int(round(crop_start_time * info.samplerate))
            start_sample = max(0, min(start_sample, info.frames))
            available_frames = info.frames - start_sample
            available_duration = (
                available_frames / info.samplerate if info.samplerate else 0.0
            )

            crop_plan[wav_path]["start_sample"] = start_sample
            crop_plan[wav_path]["crop_start_time"] = (
                start_sample / info.samplerate if info.samplerate else 0.0
            )
            crop_plan[wav_path]["available_frames"] = available_frames
            crop_plan[wav_path]["available_duration"] = available_duration

            if min_available_duration is None or available_duration < min_available_duration:
                min_available_duration = available_duration

    if min_available_duration is None or min_available_duration <= 0:
        raise ValueError("Shared timestamp leaves no remaining audio to synchronize.")

    outputs: list[str] = []
    use_shared_frame_count = len(sample_rates) == 1
    for wav_path in wav_paths:
        plan = crop_plan[wav_path]
        info = plan["info"]
        start_sample = int(plan["start_sample"])
        available_frames = int(plan["available_frames"])

        if use_shared_frame_count:
            frames_to_write = min_available_frames
        else:
            frames_to_write = min(
                available_frames,
                int(math.floor(min_available_duration * info.samplerate + 1e-9)),
            )

        if frames_to_write <= 0:
            raise ValueError(
                f"Synchronized crop length is empty for {Path(wav_path).name}"
            )

        audio, fs = sf.read(wav_path, always_2d=False)
        synced_audio = audio[start_sample : start_sample + frames_to_write]

        synced_wav_path = derive_synced_wav_path(wav_path)
        synced_txt_path = derive_synced_txt_path(wav_path)

        sf.write(
            synced_wav_path,
            synced_audio,
            fs,
            format=info.format,
            subtype=info.subtype,
            endian=info.endian,
        )
        write_synced_label_file(
            synced_txt_path,
            entries_by_file[wav_path],
            crop_start_time=float(plan["crop_start_time"]),
            crop_duration=frames_to_write / fs,
        )
        outputs.append(synced_wav_path)

    timestamp_text = shared_timestamp.strftime("%Y-%m-%d %H:%M:%S")
    return SyncResult(
        outputs=tuple(outputs),
        message=(
            f"Created {len(outputs)} synchronized WAV "
            f"{'copy' if len(outputs) == 1 else 'copies'} using shared "
            f"message timestamp {timestamp_text}."
        ),
        shared_timestamp=shared_timestamp,
    )


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FSK Audio Demodulator")
        self.geometry("520x620")
        self.resizable(False, False)

        self.frame = tk.Frame(self, padx=20, pady=20)
        self.frame.pack(fill="both", expand=True)

        # ------------------------------
        # Title
        # ------------------------------
        tk.Label(
            self.frame, text="FSK Audio Demodulator", font=("Arial", 20, "bold")
        ).pack(pady=(12, 6))

        # State
        self.selected_files: list[str] = []

        # UI vars
        self.inp = tk.StringVar()
        self.status = tk.StringVar(value="Idle")

        # FSK parameters
        self.f0_hz = tk.DoubleVar(value=20884.0)
        self.f1_hz = tk.DoubleVar(value=22274.0)

        # Other controls
        self.generate_debug = tk.BooleanVar(value=False)
        self.use_segmentation = tk.BooleanVar(value=True)
        self.minutes_per_segment = tk.IntVar(value=1)
        self.use_ecc = tk.BooleanVar(value=True)
        self.ecc_parity_bytes = tk.IntVar(value=DEFAULT_ECC_NSYM)
        self.auto_sync_audio = tk.BooleanVar(value=False)

        # Global click to defocus entry widgets
        self.bind_all("<Button-1>", self._global_defocus, add="+")

        pad = {"padx": 12, "pady": 4}

        # Input
        tk.Label(
            self.frame, text="Input audio file(s)", font=("Arial", 15, "bold")
        ).pack(anchor="w", **pad)
        row = tk.Frame(self.frame)
        row.pack(fill="x", **pad)
        ttk.Entry(
            row,
            textvariable=self.inp,
            state="readonly",
        ).pack(side="left", fill="x", expand=True)

        tk.Button(
            row,
            text="Browse",
            command=self.pick_wavs,
        ).pack(side="left", padx=(6, 0))

        # FSK parameters
        tk.Label(self.frame, text="FSK Parameters", font=("Arial", 15, "bold")).pack(
            anchor="w", padx=12, pady=(8, 2)
        )
        fsk_row = tk.Frame(self.frame)
        fsk_row.pack(fill="x", padx=12, pady=(0, 6))

        tk.Label(fsk_row, text="f0 (Hz)").pack(side="left")
        self.f0_entry = ttk.Entry(fsk_row, textvariable=self.f0_hz, width=12)
        self.f0_entry.pack(side="left", padx=(6, 14))

        tk.Label(fsk_row, text="f1 (Hz)").pack(side="left")
        self.f1_entry = ttk.Entry(fsk_row, textvariable=self.f1_hz, width=12)
        self.f1_entry.pack(side="left", padx=(6, 0))

        # Bind auto-adjust
        self.f0_entry.bind("<FocusOut>", lambda e: self.update_low_frequency())
        self.f0_entry.bind("<Return>", lambda e: self.update_low_frequency())

        self.f1_entry.bind("<FocusOut>", lambda e: self.update_high_frequency())
        self.f1_entry.bind("<Return>", lambda e: self.update_high_frequency())

        # Segmentation option
        tk.Label(
            self.frame, text="Segmentation Settings", font=("Arial", 15, "bold")
        ).pack(anchor="w", padx=12, pady=(8, 2))
        seg_frame = tk.Frame(self.frame)
        seg_frame.pack(fill="x", **pad)
        tk.Checkbutton(
            seg_frame,
            text="Enable segmentation (minutes per segment)",
            variable=self.use_segmentation,
            command=self._toggle_seg_controls,
        ).pack(side="left")
        self.mins_entry = tk.Spinbox(
            seg_frame, from_=1, to=120, width=6, textvariable=self.minutes_per_segment
        )
        self.mins_entry.pack(side="left")

        # Debug option
        tk.Label(self.frame, text="Debug Settings", font=("Arial", 15, "bold")).pack(
            anchor="w", padx=12, pady=(8, 2)
        )
        opt = tk.Frame(self.frame)
        opt.pack(fill="x", **pad)
        tk.Checkbutton(
            opt, text="Generate debug output", variable=self.generate_debug
        ).pack(side="left")

        # Batch options
        tk.Label(self.frame, text="Batch Settings", font=("Arial", 15, "bold")).pack(
            anchor="w", padx=12, pady=(8, 2)
        )
        batch = tk.Frame(self.frame)
        batch.pack(fill="x", **pad)
        self.auto_sync_check = tk.Checkbutton(
            batch,
            text="Auto-sync matching timestamps for multiple files",
            variable=self.auto_sync_audio,
        )
        self.auto_sync_check.pack(side="left")

        # ECC options
        tk.Label(self.frame, text="Error Correction", font=("Arial", 15, "bold")).pack(
            anchor="w", padx=12, pady=(8, 2)
        )
        ecc = tk.Frame(self.frame)
        ecc.pack(fill="x", **pad)
        tk.Checkbutton(
            ecc,
            text="Enable ECC",
            variable=self.use_ecc,
            command=self._toggle_ecc_controls,
        ).pack(side="left")
        tk.Label(ecc, text="Added bytes:").pack(side="left", padx=(12, 4))
        self.ecc_entry = tk.Spinbox(
            ecc,
            from_=1,
            to=254,
            width=6,
            textvariable=self.ecc_parity_bytes,
        )
        self.ecc_entry.pack(side="left")

        tk.Label(self.frame, text="").pack(pady=(4, 0))

        # Controls
        ctrl = tk.Frame(self.frame)
        ctrl.pack(fill="x", **pad)
        self.run_btn = tk.Button(
            ctrl, text="Decode and Extract", command=self.run_clicked, width=25
        )
        self.run_btn.pack(side="left", expand=True)
        self.run_btn.pack(pady=(0, 6))

        ttk.Separator(self.frame, orient="horizontal").pack(
            fill="x", padx=12, pady=(0, 6)
        )
        self.progress = ttk.Progressbar(self.frame, mode="determinate", maximum=100)
        self.progress.pack(fill="x", padx=12)
        tk.Label(self.frame, textvariable=self.status, wraplength=460).pack(
            pady=(4, 8)
        )

        # Initial toggle state
        self._toggle_seg_controls()
        self._toggle_ecc_controls()
        self._toggle_sync_controls()

        # Initial frequency auto-fix
        self.update_low_frequency()
        self.update_high_frequency()

        # Center window
        self.update_idletasks()
        w = self.winfo_width()
        h = self.winfo_height()
        x = (self.winfo_screenwidth() // 2) - (w // 2)
        y = (self.winfo_screenheight() // 2) - (h // 2)
        self.geometry(f"{w}x{h}+{x}+{y}")

        # Bring to front
        self.deiconify()
        self.update_idletasks()
        self.lift()
        self.attributes("-topmost", True)
        self.focus_force()
        self.after(1200, lambda: self.attributes("-topmost", False))

    # ------------------------------
    # Frequency helper methods
    # ------------------------------
    def tolerance_samples(
        self, target_samples: int, tolerance_percent: int = BIT_SAMPLE_TOLERANCE_PERCENT
    ) -> int:
        return int(round(target_samples * tolerance_percent / 100.0))

    def total_samples_within_tolerance(
        self,
        total_samples: int,
        target_samples: int,
        tolerance_percent: int = BIT_SAMPLE_TOLERANCE_PERCENT,
    ) -> bool:
        tol = self.tolerance_samples(target_samples, tolerance_percent)
        return (target_samples - tol) <= total_samples <= (target_samples + tol)

    def rounded_min_bit_samples(self, fs: int) -> int:
        return ((fs * MIN_BIT_US) + 500000) // 1000000

    def quantized_freq_from_samples(self, fs: int, samples_per_period: int) -> int:
        if samples_per_period <= 0:
            return 0
        return int(math.floor(fs / samples_per_period))

    def period_count_from_samples(
        self, min_bit_samples: int, samples_per_period: int
    ) -> int:
        return int(round(min_bit_samples / samples_per_period))

    def freq_diff_u16(self, a: int, b: int) -> int:
        return abs(a - b)

    def min_required_diff_hz(self, lower_freq: int) -> int:
        return 300 + (400000 // lower_freq)

    def quantized_params(self, freq: float, fs: int = FS_HZ):
        samples_per_period = max(1, int(math.floor(fs / freq)))
        min_bit_samples = self.rounded_min_bit_samples(fs)
        period_count = self.period_count_from_samples(
            min_bit_samples, samples_per_period
        )
        quantized_freq = self.quantized_freq_from_samples(fs, samples_per_period)
        total_samples = samples_per_period * period_count
        return quantized_freq, samples_per_period, period_count, total_samples

    def adjust_low_frequency_to_valid(self, low_freq: float, high_freq: float):
        min_bit_samples = self.rounded_min_bit_samples(FS_HZ)

        low_n = max(1, int(math.floor(FS_HZ / low_freq)))
        high_n = max(1, int(math.floor(FS_HZ / high_freq)))

        high_q = self.quantized_freq_from_samples(FS_HZ, high_n)
        high_p = self.period_count_from_samples(min_bit_samples, high_n)
        high_total = high_n * high_p

        while True:
            low_q = self.quantized_freq_from_samples(FS_HZ, low_n)
            low_p = self.period_count_from_samples(min_bit_samples, low_n)
            low_total = low_n * low_p

            same_periods = low_p == high_p
            enough_diff = self.freq_diff_u16(
                high_q, low_q
            ) >= self.min_required_diff_hz(low_q)
            low_timing_ok = self.total_samples_within_tolerance(
                low_total, min_bit_samples
            )
            high_timing_ok = self.total_samples_within_tolerance(
                high_total, min_bit_samples
            )

            if (
                (not same_periods)
                and enough_diff
                and low_q < high_q
                and low_timing_ok
                and high_timing_ok
            ):
                break

            candidate_low_n = low_n + 1
            candidate_low_q = self.quantized_freq_from_samples(FS_HZ, candidate_low_n)

            if candidate_low_q < FREQ_MIN:
                break

            low_n = candidate_low_n

        low_q = self.quantized_freq_from_samples(FS_HZ, low_n)
        low_p = self.period_count_from_samples(min_bit_samples, low_n)
        low_total = low_n * low_p

        return low_q, low_n, low_p, low_total

    def adjust_high_frequency_to_valid(self, low_freq: float, high_freq: float):
        min_bit_samples = self.rounded_min_bit_samples(FS_HZ)

        low_n = max(1, int(math.floor(FS_HZ / low_freq)))
        high_n = max(1, int(math.floor(FS_HZ / high_freq)))

        low_q = self.quantized_freq_from_samples(FS_HZ, low_n)
        low_p = self.period_count_from_samples(min_bit_samples, low_n)
        low_total = low_n * low_p

        while True:
            high_q = self.quantized_freq_from_samples(FS_HZ, high_n)
            high_p = self.period_count_from_samples(min_bit_samples, high_n)
            high_total = high_n * high_p

            same_periods = low_p == high_p
            enough_diff = self.freq_diff_u16(
                high_q, low_q
            ) >= self.min_required_diff_hz(low_q)
            low_timing_ok = self.total_samples_within_tolerance(
                low_total, min_bit_samples
            )
            high_timing_ok = self.total_samples_within_tolerance(
                high_total, min_bit_samples
            )

            if (
                (not same_periods)
                and enough_diff
                and high_q > low_q
                and low_timing_ok
                and high_timing_ok
            ):
                break

            if high_n <= 1:
                break

            candidate_high_n = high_n - 1
            candidate_high_q = self.quantized_freq_from_samples(FS_HZ, candidate_high_n)

            if candidate_high_q > FREQ_MAX:
                break

            high_n = candidate_high_n

        high_q = self.quantized_freq_from_samples(FS_HZ, high_n)
        high_p = self.period_count_from_samples(min_bit_samples, high_n)
        high_total = high_n * high_p

        return high_q, high_n, high_p, high_total

    def update_low_frequency(self):
        try:
            low = float(self.f0_hz.get())
            high = float(self.f1_hz.get())
        except Exception:
            return

        low = max(FREQ_MIN, min(FREQ_MAX, low))
        high = max(FREQ_MIN, min(FREQ_MAX, high))

        if low >= high:
            low = high - 1

        if low < FREQ_MIN:
            low = FREQ_MIN

        new_low, _, _, _ = self.adjust_low_frequency_to_valid(low, high)
        self.f0_hz.set(int(new_low))

    def update_high_frequency(self):
        try:
            low = float(self.f0_hz.get())
            high = float(self.f1_hz.get())
        except Exception:
            return

        low = max(FREQ_MIN, min(FREQ_MAX, low))
        high = max(FREQ_MIN, min(FREQ_MAX, high))

        if high <= low:
            high = low + 1

        if high > FREQ_MAX:
            high = FREQ_MAX

        new_high, _, _, _ = self.adjust_high_frequency_to_valid(low, high)
        self.f1_hz.set(int(new_high))

    # ------------------------------
    # UI helpers
    # ------------------------------
    def _global_defocus(self, event):
        w = event.widget

        # Keep normal behavior when clicking into editable input widgets
        if isinstance(w, (tk.Entry, ttk.Entry, tk.Spinbox, tk.Text)):
            return

        # Let the clicked widget process the click first, then remove focus
        self.after_idle(self.focus_set)

    def _toggle_seg_controls(self):
        enabled = self.use_segmentation.get()
        state = "normal" if enabled else "disabled"
        self.mins_entry.config(state=state)

    def _toggle_ecc_controls(self):
        enabled = self.use_ecc.get()
        state = "normal" if enabled else "disabled"
        self.ecc_entry.config(state=state)

    def _toggle_sync_controls(self):
        enabled = len(self.selected_files) >= 2
        if not enabled:
            self.auto_sync_audio.set(False)
        self.auto_sync_check.config(state="normal" if enabled else "disabled")

    def _get_fsk_params(self) -> tuple[float, float]:
        try:
            f0 = float(self.f0_hz.get())
            f1 = float(self.f1_hz.get())
        except Exception:
            raise ValueError("f0 and f1 must be valid numbers (Hz)")

        if not (f0 > 0 and f1 > 0):
            raise ValueError("f0 and f1 must be > 0 Hz")
        if abs(f0 - f1) < 1e-9:
            raise ValueError("f0 and f1 must be different")

        return f0, f1

    def pick_wavs(self):
        paths = filedialog.askopenfilenames(
            title="Select WAV file(s)", filetypes=[("WAV files", "*.wav *.WAV")]
        )
        if not paths:
            return
        self.selected_files = list(paths)
        if len(self.selected_files) == 1:
            self.inp.set(Path(self.selected_files[0]).name)
        else:
            self.inp.set(f"{len(self.selected_files)} files selected")
        self._toggle_sync_controls()

    def run_clicked(self):
        if not self.selected_files:
            messagebox.showerror("Error", "Please choose at least one input WAV")
            return

        try:
            f0, f1 = self._get_fsk_params()
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        gen_debug = self.generate_debug.get()
        seg_minutes = (
            self.minutes_per_segment.get() if self.use_segmentation.get() else -1
        )
        use_ecc = self.use_ecc.get()
        parity_bytes = self.ecc_parity_bytes.get()
        auto_sync = self.auto_sync_audio.get() and len(self.selected_files) >= 2

        if use_ecc and not (1 <= parity_bytes <= 254):
            messagebox.showerror("Error", "Parity bytes must be between 1 and 254")
            return

        self.run_btn.config(state="disabled")
        self.progress["value"] = 0
        self.status.set("Running...")

        def worker():
            errors = []
            sync_result: SyncResult | None = None
            sync_error: str | None = None
            failed_paths = set()
            total_messages = 0

            def set_status(text: str):
                self.after(0, lambda value=text: self.status.set(value))

            def set_progress(value: float):
                self.after(0, lambda v=value: self.progress.configure(value=v))

            def show_dialog(kind: str, title: str, text: str):
                dialog = getattr(messagebox, kind)
                self.after(0, lambda: dialog(title, text))

            try:
                total = len(self.selected_files)
                for i, wav_path in enumerate(self.selected_files, start=1):
                    name = Path(wav_path).name

                    # Progress callback: maps per-file [0,1] to overall progress
                    def make_progress_cb(file_idx, file_total, file_name):
                        def cb(fraction, message):
                            overall = ((file_idx - 1) + fraction) / file_total * 100
                            set_progress(overall)
                            set_status(f"[{file_idx}/{file_total}] {file_name}: {message}")
                        return cb

                    progress_cb = make_progress_cb(i, total, name)
                    progress_cb(0.0, "Starting")
                    try:
                        msg_count = decode_fsk(
                            wav_path,
                            f0=f0,
                            f1=f1,
                            generate_debug=gen_debug,
                            minutes_per_segment=seg_minutes,
                            use_ecc=use_ecc,
                            ecc_nsym=parity_bytes,
                            progress_callback=progress_cb,
                        )
                        total_messages += msg_count if msg_count else 0
                    except Exception as e:
                        errors.append((wav_path, str(e)))
                        failed_paths.add(wav_path)

                successful_paths = [
                    wav_path
                    for wav_path in self.selected_files
                    if wav_path not in failed_paths
                ]

                if auto_sync and len(successful_paths) >= 2:
                    set_status("Synchronizing matching audio files...")
                    try:
                        sync_result = synchronize_audio_files(successful_paths)
                    except Exception as e:
                        sync_error = str(e)
                elif auto_sync and len(successful_paths) < 2:
                    sync_result = SyncResult(
                        outputs=(),
                        message=(
                            "Automatic synchronization was skipped because fewer "
                            "than two files were decoded successfully."
                        ),
                    )

                if errors:
                    msg = "Some files failed:\n\n" + "\n".join(
                        f"- {Path(p).name}: {err}" for p, err in errors
                    )
                    show_dialog("showerror", "Completed with errors", msg)

                msg_summary = (
                    f"Found {total_messages} "
                    f"{'message' if total_messages == 1 else 'messages'}."
                )

                if errors and sync_error:
                    set_status(f"Done ({len(errors)} errors). {msg_summary} Sync failed: {sync_error}")
                elif errors and sync_result is not None:
                    set_status(f"Done ({len(errors)} errors). {msg_summary} {sync_result.message}")
                elif errors:
                    set_status(f"Done ({len(errors)} errors). {msg_summary}")
                elif sync_error:
                    set_status(f"Done. {msg_summary} Sync failed: {sync_error}")
                elif sync_result is not None and sync_result.outputs:
                    set_status(f"Done. {msg_summary} {sync_result.message}")
                elif sync_result is not None:
                    set_status(f"Done. {msg_summary} {sync_result.message}")
                else:
                    set_status(f"Done. {msg_summary}")
            finally:
                set_progress(100)
                self.after(0, lambda: self.run_btn.config(state="normal"))

                def reset_ui():
                    self.status.set("")
                    self.progress["value"] = 0

                self.after(10000, reset_ui)

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    App().mainloop()
