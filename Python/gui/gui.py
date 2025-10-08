import os
import sys
import subprocess
import shutil
import tempfile
import tkinter as tk
from tkinter import messagebox
import threading
import tkinter.scrolledtext as st

# Import scripts
from scripts.paths import *
from scripts.user_config import *
from scripts.build import *

# ---------------------------------------------------------
# GUI Setup
# ---------------------------------------------------------

root = tk.Tk()
root.title("STM32 Builder & Flasher")

frame = tk.Frame(root, padx=20, pady=20)
frame.pack()

# ------------------------------
# Title
# ------------------------------
tk.Label(frame, text="STM32 Build & Flash Tool", font=("Arial", 14, "bold")).grid(
    row=0, column=0, columnspan=2, pady=10
)


# ------------------------------
# Input fields (label + entry per row)
# ------------------------------
def make_labeled_entry(parent, label, value, row):
    tk.Label(parent, text=label + ":").grid(row=row, column=0, sticky="e", padx=(0, 10))
    entry = tk.Entry(parent, width=25)
    entry.grid(row=row, column=1, sticky="w")
    entry.insert(0, value)
    return entry


root.user_string_field = make_labeled_entry(
    frame, "User String", read_user_config_value("USER_STRING"), 1
)
root.device_id_field = make_labeled_entry(
    frame, "Device ID", int(read_user_config_value("DEVICE_ID")), 2
)
root.location_field = make_labeled_entry(
    frame, "Location", read_user_config_value("LOCATION"), 3
)
root.temperature_field = make_labeled_entry(
    frame, "Temperature", int(read_user_config_value("TEMPERATURE")), 4
)

# ------------------------------
# Include checkboxes
# ------------------------------
tk.Label(frame, text="Include in Watermark:", font=("Arial", 10, "bold")).grid(
    row=5, column=0, columnspan=2, sticky="w", pady=(10, 0)
)

include_frame = tk.Frame(frame)
include_frame.grid(row=6, column=0, columnspan=2, sticky="w")

root.include_user_string_var = tk.IntVar(value=1)
root.include_device_id_var = tk.IntVar(value=1)
root.include_location_var = tk.IntVar(value=1)
root.include_temperature_var = tk.IntVar(value=1)
root.include_time_var = tk.IntVar(value=1)

# Two-column grid for checkboxes
tk.Checkbutton(
    include_frame, text="User String", variable=root.include_user_string_var
).grid(row=0, column=0, sticky="w")
tk.Checkbutton(
    include_frame, text="Device ID", variable=root.include_device_id_var
).grid(row=1, column=0, sticky="w")
tk.Checkbutton(include_frame, text="Location", variable=root.include_location_var).grid(
    row=2, column=0, sticky="w"
)
tk.Checkbutton(
    include_frame, text="Temperature", variable=root.include_temperature_var
).grid(row=0, column=1, sticky="w", padx=20)
tk.Checkbutton(include_frame, text="Timestamp", variable=root.include_time_var).grid(
    row=1, column=1, sticky="w", padx=20
)

# ------------------------------
# Interval controls
# ------------------------------
default_interval_var = tk.IntVar(value=1)

interval_frame = tk.Frame(frame)
interval_frame.grid(row=7, column=0, columnspan=2, pady=(10, 5), sticky="w")

default_interval_cb = tk.Checkbutton(
    interval_frame, text="Use default interval", variable=default_interval_var
)
default_interval_cb.grid(row=0, column=0, sticky="w")

interval_field = tk.Entry(interval_frame, width=10)
interval_field.grid(row=0, column=1, padx=(10, 0))
interval_field.insert(
    0, int(read_user_config_value("INTERVAL_BETWEEN_REPEATS_MINUTES"))
)


def toggle_interval_field():
    interval_field.config(state="disabled" if default_interval_var.get() else "normal")


default_interval_var.trace_add("write", lambda *args: toggle_interval_field())
toggle_interval_field()

# ------------------------------
# Log checkbox
# ------------------------------
show_log_var = tk.IntVar(value=0)
tk.Checkbutton(frame, text="Show build log", variable=show_log_var).grid(
    row=8, column=0, columnspan=2, sticky="w", pady=(10, 0)
)

# ------------------------------
# Build button
# ------------------------------
build_btn = tk.Button(
    frame,
    text="🔨⚡ Build & Flash",
    width=25,
    command=lambda: dual_build_flash(
        root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET
    ),
)
build_btn.grid(row=9, column=0, columnspan=2, pady=20)

root.mainloop()
