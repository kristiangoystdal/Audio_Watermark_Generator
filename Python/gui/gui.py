import os
import sys
import subprocess
import shutil
import tempfile
import tkinter as tk
from tkinter import messagebox
import tkinter.scrolledtext as st
import threading

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
root.default_interval_var = tk.IntVar(value=1)

interval_frame = tk.Frame(frame)
interval_frame.grid(row=7, column=0, columnspan=2, pady=(10, 5), sticky="w")

default_interval_cb = tk.Checkbutton(
    interval_frame, text="Use default interval", variable=root.default_interval_var
)
default_interval_cb.grid(row=0, column=0, sticky="w")

root.interval_field = tk.Entry(interval_frame, width=10)
root.interval_field.grid(row=0, column=1, padx=(10, 0))
root.interval_field.insert(
    0, int(read_user_config_value("INTERVAL_BETWEEN_REPEATS_MINUTES"))
)

# -------------------------------
# Delay initial timestamp setting
# -------------------------------
root.default_delay_var = tk.IntVar(value=0)

delay_frame = tk.Frame(frame)
delay_frame.grid(row=8, column=0, columnspan=2, sticky="w")
default_delay_cb = tk.Checkbutton(
    delay_frame,
    text="Start at a specific minute (0-59)",
    variable=root.default_delay_var,
)
default_delay_cb.grid(row=0, column=0, sticky="w")

root.delay_field = tk.Entry(delay_frame, width=10)
root.delay_field.grid(row=0, column=1, padx=(10, 0))
root.delay_field.insert(0, int(read_user_config_value("STARTING_MINUTE")))

# ------------------------------
# Frequency Settings Frame
# ------------------------------
frequency_frame = tk.Frame(frame)
frequency_frame.grid(row=10, column=0, columnspan=2, pady=(10, 0), sticky="w")

tk.Label(
    frequency_frame, text="FSK Frequency Settings", font=("Arial", 10, "bold")
).grid(row=0, column=0, sticky="w", pady=(0, 5))

# Dropdown menu for frequency pairs
frequency_pairs = read_frequency_pairs()
if not frequency_pairs:
    frequency_pairs = [
        (20833.33, 22222.22),
        (11111.11, 12500.00),
        (6944.44, 8333.33),
        (1388.88, 2777.77),
    ]

root.frequency_pair_var = tk.StringVar(value="1")

# Map display text to 1-based index
pair_display_map = {
    f"{f0:.2f} Hz & {f1:.2f} Hz": str(i + 1)
    for i, (f0, f1) in enumerate(frequency_pairs)
}

# The text shown in the dropdown
menu_display_var = tk.StringVar(
    value=f"{frequency_pairs[0][0]:.2f} Hz & {frequency_pairs[0][1]:.2f} Hz"
)

# Dropdown
frequency_menu = tk.OptionMenu(
    frequency_frame,
    menu_display_var,
    *pair_display_map.keys(),
)
frequency_menu.grid(row=1, column=0, sticky="w", pady=(0, 5))


# When user selects a new pair, store its 1-based index in the variable
def on_pair_select(selection):
    root.frequency_pair_var.set(pair_display_map[selection])
    menu_display_var.set(selection)


# Rebuild dropdown with callbacks
frequency_menu["menu"].delete(0, "end")
for display_text in pair_display_map:
    frequency_menu["menu"].add_command(
        label=display_text,
        command=lambda v=display_text: on_pair_select(v),
    )


def toggle_interval_field():
    root.interval_field.config(
        state="disabled" if root.default_interval_var.get() else "normal"
    )


def toggle_delay_field():
    root.delay_field.config(
        state="disabled" if not root.default_delay_var.get() else "normal"
    )


# Initial toggle based on default values
root.default_interval_var.trace_add("write", lambda *args: toggle_interval_field())
toggle_interval_field()
root.default_delay_var.trace_add("write", lambda *args: toggle_delay_field())
toggle_delay_field()

# ------------------------------
# Log checkbox
# ------------------------------
show_log_frame = tk.Frame(frame)
show_log_frame.grid(row=11, column=0, columnspan=2, sticky="w")
show_log_var = tk.IntVar(value=0)
tk.Checkbutton(show_log_frame, text="Show build log", variable=show_log_var).grid(
    row=0, column=0, sticky="w"
)


# ------------------------------
# Background build logic (non-blocking)
# ------------------------------
def start_dual_flash_thread():
    """Run build + flash in background thread (so GUI stays responsive)."""
    build_btn.config(state="disabled", text="⏳ Building...")
    threading.Thread(
        target=lambda: safe_dual_flash_call(),
        daemon=True,
    ).start()


def safe_dual_flash_call():
    """Thread-safe wrapper for dual_build_flash with Tk messageboxes."""
    try:
        dual_build_flash(
            root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET
        )
        root.after(
            0,
            lambda: messagebox.showinfo(
                "Build & Flash", "✅ Build and Flash completed successfully!"
            ),
        )
    except Exception as e:
        root.after(0, lambda: messagebox.showerror("Error", f"❌ {e}"))
    finally:
        root.after(
            0, lambda: build_btn.config(state="normal", text="🔨⚡ Build & Flash")
        )


# ------------------------------
# Build button
# ------------------------------
build_btn_frame = tk.Frame(frame)
build_btn_frame.grid(row=12, column=0, columnspan=2)
build_btn = tk.Button(
    build_btn_frame,
    text="🔨⚡ Build & Flash",
    width=25,
    command=start_dual_flash_thread,
)
build_btn.grid(row=9, column=0, columnspan=2, pady=20)

root.mainloop()
