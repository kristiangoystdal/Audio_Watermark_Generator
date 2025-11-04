import tkinter as tk
from tkinter import messagebox
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
root.resizable(False, False)

frame = tk.Frame(root, height=400, width=600, padx=20, pady=20)
frame.pack()

# ------------------------------
# Title
# ------------------------------
tk.Label(frame, text="STM32 Build & Flash Tool", font=("Arial", 20, "bold")).grid(
    row=0, column=0, columnspan=2, pady=10
)


# ------------------------------
# Input fields
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

# ------------------------------
# Include checkboxes
# ------------------------------
tk.Label(frame, text="Include in Watermark:", font=("Arial", 15, "bold")).grid(
    row=5, column=0, columnspan=2, sticky="w", pady=(10, 0)
)

include_frame = tk.Frame(frame)
include_frame.grid(row=6, column=0, columnspan=2, sticky="w")

root.include_user_string_var = tk.IntVar(value=1)
root.include_device_id_var = tk.IntVar(value=1)
root.include_location_var = tk.IntVar(value=1)
root.include_temperature_var = tk.IntVar(value=1)
root.include_time_var = tk.IntVar(value=1)

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

tk.Label(interval_frame, text="Interval (minutes):", font=("Arial", 15, "bold")).grid(
    row=0, column=0, columnspan=2, sticky="w", pady=(10, 0)
)

default_interval_cb = tk.Checkbutton(
    interval_frame,
    text="Use default interval (1)",
    variable=root.default_interval_var,
)
default_interval_cb.grid(row=1, column=0, sticky="w")

root.interval_field = tk.Entry(interval_frame, width=10)
root.interval_field.grid(row=1, column=1, padx=(10, 0))
root.interval_field.insert(
    0, int(read_user_config_value("INTERVAL_BETWEEN_REPEATS_MINUTES"))
)

# -------------------------------
# Delay initial timestamp setting
# -------------------------------
root.default_delay_var = tk.IntVar(value=0)

delay_frame = tk.Frame(frame)
delay_frame.grid(row=8, column=0, columnspan=2, sticky="w")

tk.Label(delay_frame, text="Initial Delay (minutes):", font=("Arial", 15, "bold")).grid(
    row=0, column=0, columnspan=2, sticky="w", pady=(10, 0)
)

default_delay_cb = tk.Checkbutton(
    delay_frame,
    text="Start at a specific minute (0-59)",
    variable=root.default_delay_var,
)
default_delay_cb.grid(row=1, column=0, sticky="w")

root.delay_field = tk.Entry(delay_frame, width=10)
root.delay_field.grid(row=1, column=1, padx=(10, 0))
root.delay_field.insert(0, int(read_user_config_value("STARTING_MINUTE")))

# ------------------------------
# Frequency Settings Frame
# ------------------------------
frequency_frame = tk.Frame(frame)
frequency_frame.grid(row=10, column=0, columnspan=2, pady=(10, 5), sticky="w")

tk.Label(
    frequency_frame, text="FSK Frequency Settings", font=("Arial", 15, "bold")
).grid(row=0, column=0, sticky="w", pady=(0, 5))

frequency_pairs = read_frequency_pairs()
if not frequency_pairs:
    frequency_pairs = [
        (20833.33, 22222.22),
        (11111.11, 12500.00),
        (6944.44, 8333.33),
        (1388.88, 2777.77),
    ]

root.frequency_pair_var = tk.StringVar(value="1")
pair_display_map = {
    f"{f0:.2f} Hz & {f1:.2f} Hz": str(i + 1)
    for i, (f0, f1) in enumerate(frequency_pairs)
}

menu_display_var = tk.StringVar(
    value=f"{frequency_pairs[0][0]:.2f} Hz & {frequency_pairs[0][1]:.2f} Hz"
)

frequency_menu = tk.OptionMenu(
    frequency_frame,
    menu_display_var,
    *pair_display_map.keys(),
)
frequency_menu.grid(row=1, column=0, sticky="w", pady=(0, 5))


def on_pair_select(selection):
    root.frequency_pair_var.set(pair_display_map[selection])
    menu_display_var.set(selection)


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


root.default_interval_var.trace_add("write", lambda *args: toggle_interval_field())
toggle_interval_field()
root.default_delay_var.trace_add("write", lambda *args: toggle_delay_field())
toggle_delay_field()

# ------------------------------
# Log checkbox
# ------------------------------
show_log_frame = tk.Frame(frame)
show_log_frame.grid(row=11, column=0, columnspan=2, pady=(10, 5), sticky="w")

tk.Label(show_log_frame, text="Debug Settings", font=("Arial", 15, "bold")).grid(
    row=0, column=0, sticky="w", pady=(0, 5)
)

show_log_var = tk.IntVar(value=0)
tk.Checkbutton(show_log_frame, text="Show build log", variable=show_log_var).grid(
    row=1, column=0, sticky="w"
)


# ------------------------------
# Validation logic
# ------------------------------
def is_field_valid(field, var_type, min_val=None, max_val=None):
    val = field.get().strip()
    if not val:
        return False
    try:
        if var_type == int:
            val = int(val)
        elif var_type == float:
            val = float(val)
        elif var_type == str:
            if not val:
                return False
            elif field == root.user_string_field and len(val.strip('"')) > 48:
                return False
            elif field == root.location_field and len(val.strip('"')) > 18:
                return False
            return True
    except ValueError:
        return False
    if min_val is not None and val < min_val:
        return False
    if max_val is not None and val > max_val:
        return False
    return True


def validate_all_fields():
    invalid_fields = []
    if not is_field_valid(root.user_string_field, str):
        invalid_fields.append("User String must be 1-48 characters")
    if not is_field_valid(root.device_id_field, int, 0, 99):
        invalid_fields.append("Device ID must be an integer between 0 and 99")
    if not is_field_valid(root.location_field, str):
        invalid_fields.append("Location must be 1-18 characters")
    if (
        not is_field_valid(root.interval_field, int, 1, 1440)
        and not root.default_interval_var.get()
    ):
        invalid_fields.append("Interval must be an integer between 1 and 1440 minutes")
    if (
        not is_field_valid(root.delay_field, int, 0, 59)
        and root.default_delay_var.get()
    ):
        invalid_fields.append(
            "Initial Delay must be an integer between 0 and 59 minutes"
        )

    if invalid_fields:
        formatted = "\n".join(f"• {msg}" for msg in invalid_fields)
        messagebox.showwarning(
            "Invalid Input",
            f"⚠️ Please correct the following fields:\n\n{formatted}",
        )
        return False
    return True


# ------------------------------
# Build button and status label
# ------------------------------
build_btn_frame = tk.Frame(frame)
build_btn_frame.grid(row=12, column=0, columnspan=2, pady=20)

build_btn = tk.Button(
    build_btn_frame,
    text="Build & Flash",
    width=25,
    command=lambda: start_dual_flash_thread(),
)
build_btn.pack()

status_label = tk.Label(build_btn_frame, text="", font=("Arial", 14, "italic"))
status_label.pack(pady=(10, 0))


# ------------------------------
# Background build logic
# ------------------------------
def start_dual_flash_thread():
    if not validate_all_fields():
        return

    build_btn.config(state="disabled")
    status_label.config(text="⏳ Building... Please wait.")

    threading.Thread(target=dual_build_flash_call, daemon=True).start()


def dual_build_flash_call():
    try:
        dual_build_flash(
            root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET
        )
        root.after(
            0,
            lambda: [
                status_label.config(text="✅ Build & Flash completed successfully!"),
                messagebox.showinfo(
                    "Build & Flash", "✅ Build and Flash completed successfully!"
                ),
            ],
        )
    except Exception as e:
        root.after(
            0,
            lambda: [
                status_label.config(text="❌ Build failed"),
                messagebox.showerror("Error", f"❌ {e}"),
            ],
        )
    finally:
        root.after(0, lambda: build_btn.config(state="normal"))
        root.after(4000, lambda: status_label.config(text=""))


# Optional: Bind Enter to build
root.bind("<Return>", lambda e: start_dual_flash_thread())

root.mainloop()
