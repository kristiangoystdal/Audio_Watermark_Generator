#!/usr/bin/env python3
import os
import math
import threading
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from pathlib import Path

from demodulator import decode_fsk
from reed_solomon import NSYM as DEFAULT_ECC_NSYM

# ------------------------------
# Frequency helpers constants
# ------------------------------
FS_HZ = 960000
MIN_BIT_US = 3000
FREQ_MIN = 1000
FREQ_MAX = 24000
BIT_SAMPLE_TOLERANCE_PERCENT = 1


def derive_txt_path(wav_path: str) -> str:
    base, _ = os.path.splitext(wav_path)
    return base + ".txt"


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FSK Audio Demodulator")
        self.geometry("520x560")
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
        self.progress = ttk.Progressbar(self.frame, mode="indeterminate")
        self.progress.pack(fill="x", padx=12)
        tk.Label(self.frame, textvariable=self.status).pack(pady=(4, 8))

        # Initial toggle state
        self._toggle_seg_controls()
        self._toggle_ecc_controls()

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

        if use_ecc and not (1 <= parity_bytes <= 254):
            messagebox.showerror("Error", "Parity bytes must be between 1 and 254")
            return

        self.run_btn.config(state="disabled")
        self.progress.start(12)
        self.status.set("Running...")

        def worker():
            errors = []
            try:
                total = len(self.selected_files)
                for i, wav_path in enumerate(self.selected_files, start=1):
                    name = Path(wav_path).name
                    self.status.set(f"Processing {i}/{total}: {name}")
                    try:
                        decode_fsk(
                            wav_path,
                            f0=f0,
                            f1=f1,
                            generate_debug=gen_debug,
                            minutes_per_segment=seg_minutes,
                            use_ecc=use_ecc,
                            ecc_nsym=parity_bytes,
                        )
                    except Exception as e:
                        errors.append((wav_path, str(e)))

                if errors:
                    msg = "Some files failed:\n\n" + "\n".join(
                        f"- {Path(p).name}: {err}" for p, err in errors
                    )
                    messagebox.showerror("Completed with errors", msg)

                self.status.set(
                    "Done" if not errors else f"Done ({len(errors)} errors)"
                )
            finally:
                self.progress.stop()
                self.run_btn.config(state="normal")

                def reset_ui():
                    self.status.set("")
                    self.progress["value"] = 0

                self.after(4000, reset_ui)

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    App().mainloop()
