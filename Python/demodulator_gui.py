#!/usr/bin/env python3
import os
import threading
import tkinter as tk
from tkinter import filedialog, messagebox
from pathlib import Path

from demodulator import decode_fsk  # make sure this is your module name


def derive_txt_path(wav_path: str) -> str:
    base, _ = os.path.splitext(wav_path)
    return base + ".txt"


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FSK Demodulator")
        self.geometry("520x260")
        self.resizable(False, False)

        # Presets: one dropdown controls all three parameters
        self.presets = {
            "Frequency Pair 1": {"f0": 20833.33, "f1": 22222.22, "p0": 60},
            "Frequency Pair 2": {"f0": 11111.11, "f1": 12500.00, "p0": 32},
            "Frequency Pair 3": {"f0": 6944.44, "f1": 8333.33, "p0": 20},
            "Frequency Pair 4": {"f0": 1388.88, "f1": 2777.77, "p0": 4},
        }

        self.inp = tk.StringVar()
        self.outp = tk.StringVar()
        self.selected_preset = tk.StringVar(value="Frequency Pair 1")
        self.status = tk.StringVar(value="Idle")

        pad = {"padx": 12, "pady": 4}

        tk.Label(self, text="Input WAV:").pack(anchor="w", **pad)
        row = tk.Frame(self)
        row.pack(fill="x", **pad)
        tk.Entry(row, textvariable=self.inp).pack(side="left", fill="x", expand=True)
        tk.Button(row, text="Browse", command=self.pick_wav).pack(
            side="left", padx=(6, 0)
        )

        tk.Label(self, text="Output TXT (auto):").pack(anchor="w", **pad)
        tk.Entry(self, textvariable=self.outp, state="readonly").pack(fill="x", **pad)

        tk.Label(self, text="Preset:").pack(anchor="w", padx=12, pady=(8, 2))
        tk.OptionMenu(self, self.selected_preset, *self.presets.keys()).pack(
            fill="x", padx=12
        )
        self.preset_label = tk.Label(self, text=self._preset_text(), fg="gray")
        self.preset_label.pack(anchor="w", padx=12, pady=(2, 6))
        self.selected_preset.trace_add(
            "write", lambda *args: self._update_preset_label()
        )

        row3 = tk.Frame(self)
        row3.pack(**pad)
        self.run_btn = tk.Button(row3, text="Run", command=self.run_clicked)
        self.run_btn.pack(side="left")
        tk.Label(self, textvariable=self.status).pack()

    def _preset_text(self):
        p = self.presets[self.selected_preset.get()]
        return f"f0 = {p['f0']:.2f} Hz,  f1 = {p['f1']:.2f} Hz,  p0 = {p['p0']} cycles"

    def _update_preset_label(self):
        self.preset_label.config(text=self._preset_text())

    def pick_wav(self):
        path = filedialog.askopenfilename(
            title="Select WAV file", filetypes=[("WAV files", "*.wav *.WAV")]
        )
        if path:
            self.inp.set(path)
            out_full = derive_txt_path(path)
            self.outp.set(Path(out_full).name)  # show only filename
            self._output_fullpath = out_full  # keep full path internally

    def run_clicked(self):
        input_path = self.inp.get().strip()
        if not input_path:
            messagebox.showerror("Error", "Please choose an input WAV")
            return

        preset = self.presets[self.selected_preset.get()]
        f0, f1, p0 = preset["f0"], preset["f1"], preset["p0"]

        out_full = getattr(self, "_output_fullpath", derive_txt_path(input_path))
        self.outp.set(Path(out_full).name)

        self.run_btn.config(state="disabled")
        self.status.set("Running...")

        def worker():
            try:
                decode_fsk(input_path, out_full, f0=f0, f1=f1, p0=p0)
                self.status.set("Done")
                messagebox.showinfo("Success", f"Saved:\n{out_full}")
            except Exception as e:
                self.status.set("Error")
                messagebox.showerror("Error", str(e))
            finally:
                self.run_btn.config(state="normal")

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    App().mainloop()
