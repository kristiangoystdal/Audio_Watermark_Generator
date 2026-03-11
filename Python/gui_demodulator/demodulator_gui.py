#!/usr/bin/env python3
import os
import threading
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from pathlib import Path

from demodulator import decode_fsk
from reed_solomon import NSYM as DEFAULT_ECC_NSYM


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

        # Presets
        self.presets = {
            "Frequency Pair 1": {"f0": 20884, "f1": 22274, "p0": 60},
            "Frequency Pair 2": {"f0": 11133, "f1": 12525, "p0": 32},
            "Frequency Pair 3": {"f0": 6957, "f1": 8350, "p0": 20},
            "Frequency Pair 4": {"f0": 1392, "f1": 2781, "p0": 4},
        }

        # State
        self.selected_files: list[str] = []

        # UI vars
        self.inp = tk.StringVar()
        self.selected_preset = tk.StringVar(value="Frequency Pair 1")
        self.status = tk.StringVar(value="Idle")

        # New controls
        self.generate_debug = tk.BooleanVar(value=False)
        self.use_segmentation = tk.BooleanVar(value=True)
        self.minutes_per_segment = tk.IntVar(value=1)
        self.use_ecc = tk.BooleanVar(value=True)
        self.ecc_parity_bytes = tk.IntVar(value=DEFAULT_ECC_NSYM)

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

        # Preset
        tk.Label(self.frame, text="FSK Parameters", font=("Arial", 15, "bold")).pack(
            anchor="w", padx=12, pady=(8, 2)
        )
        tk.OptionMenu(self.frame, self.selected_preset, *self.presets.keys()).pack(
            fill="x", padx=12
        )
        self.preset_label = tk.Label(self.frame, text=self._preset_text(), fg="gray")
        self.preset_label.pack(anchor="w", padx=12, pady=(2, 6))
        self.selected_preset.trace_add("write", lambda *_: self._update_preset_label())

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
        tk.Label(
            self.frame, text="Error Correction", font=("Arial", 15, "bold")
        ).pack(anchor="w", padx=12, pady=(8, 2))
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

        # Add spacing
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

        # Center window
        self.update_idletasks()
        w = self.winfo_width()
        h = self.winfo_height()
        x = (self.winfo_screenwidth() // 2) - (w // 2)
        y = (self.winfo_screenheight() // 2) - (h // 2)
        self.geometry(f"{w}x{h}+{x}+{y}")

        # Bring to front (Option A)
        self.deiconify()
        self.update_idletasks()
        self.lift()
        self.attributes("-topmost", True)
        self.focus_force()
        self.after(1200, lambda: self.attributes("-topmost", False))

    # Helpers
    def _preset_text(self):
        p = self.presets[self.selected_preset.get()]
        return f"f0 = {p['f0']:.2f} Hz,  f1 = {p['f1']:.2f} Hz,  p0 = {p['p0']} cycles"

    def _update_preset_label(self):
        self.preset_label.config(text=self._preset_text())

    def _toggle_seg_controls(self):
        enabled = self.use_segmentation.get()
        state = "normal" if enabled else "disabled"
        self.mins_entry.config(state=state)

    def _toggle_ecc_controls(self):
        enabled = self.use_ecc.get()
        state = "normal" if enabled else "disabled"
        self.ecc_entry.config(state=state)

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

        preset = self.presets[self.selected_preset.get()]
        f0, f1, p0 = preset["f0"], preset["f1"], preset["p0"]

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
                            p0=p0,
                            generate_debug=gen_debug,
                            minutes_per_segment=seg_minutes,
                            use_ecc=use_ecc,
                            ecc_nsym=parity_bytes,
                        )

                    except Exception as e:
                        errors.append((wav_path, str(e)))
                # Quiet on success; show error popup if any failed
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
