import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
import threading
import math

# Import scripts
from scripts.paths import *
from scripts.user_config import *
from scripts.build import *


# ---------------------------------------------------------
# GUI Setup
# ---------------------------------------------------------
root = tk.Tk()
root.title("Audio Watermark Flash Tool")
root.resizable(False, False)
root.geometry("1000x760")

outer_frame = tk.Frame(root)
outer_frame.pack(fill="both", expand=True)

main_frame = tk.Frame(outer_frame, padx=24, pady=24)
main_frame.place(relx=0.5, y=0, anchor="n")

# ------------------------------
# Title
# ------------------------------
tk.Label(
    main_frame,
    text="Audio Watermark Flash Tool",
    font=("Arial", 22, "bold"),
).grid(row=0, column=0, columnspan=2, pady=(0, 20), sticky="")

# ------------------------------
# Two-column content area
# ------------------------------
content_frame = tk.Frame(main_frame)
content_frame.grid(row=1, column=0, columnspan=2)

content_frame.grid_columnconfigure(0, weight=1)
content_frame.grid_columnconfigure(1, weight=1)
content_frame.grid_anchor("n")

left_col = tk.Frame(content_frame)
right_col = tk.Frame(content_frame)

left_col.grid(row=0, column=0, padx=(0, 28), sticky="n")
right_col.grid(row=0, column=1, sticky="n")


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
SECTION_WIDTH = 470
ENTRY_WIDTH = 28
SMALL_ENTRY_WIDTH = 12


def make_section(parent, title, row):
    section = tk.LabelFrame(
        parent,
        text=title,
        padx=14,
        pady=12,
        font=("Arial", 11, "bold"),
        width=SECTION_WIDTH,
    )
    section.grid(row=row, column=0, sticky="ew", pady=(0, 14))
    return section


def make_labeled_entry(parent, label, value, row, width=ENTRY_WIDTH):
    tk.Label(parent, text=label + ":").grid(
        row=row, column=0, sticky="e", padx=(0, 12), pady=6
    )
    entry = tk.Entry(parent, width=width)
    entry.grid(row=row, column=1, sticky="w", pady=6)
    entry.insert(0, str(value))
    return entry


# ------------------------------
# Input fields
# ------------------------------
input_frame = make_section(left_col, "Device Information", 0)

root.user_string_field = make_labeled_entry(
    input_frame, "User String", read_user_config_value("USER_STRING"), 0
)
root.device_id_field = make_labeled_entry(
    input_frame, "Device ID", int(read_user_config_value("DEVICE_ID")), 1
)
root.location_field = make_labeled_entry(
    input_frame, "Location", read_user_config_value("LOCATION"), 2
)

# ------------------------------
# Include checkboxes
# ------------------------------
include_frame = make_section(right_col, "Include in Watermark", 0)

checkbox_grid = tk.Frame(include_frame)
checkbox_grid.grid(row=0, column=0, sticky="w", pady=(4, 0))

root.include_user_string_var = tk.IntVar(value=1)
root.include_device_id_var = tk.IntVar(value=1)
root.include_location_var = tk.IntVar(value=1)
root.include_temperature_var = tk.IntVar(value=1)
root.include_time_var = tk.IntVar(value=1)

tk.Checkbutton(
    checkbox_grid, text="User String", variable=root.include_user_string_var
).grid(row=0, column=0, sticky="w", padx=(0, 30), pady=4)

tk.Checkbutton(
    checkbox_grid, text="Temperature", variable=root.include_temperature_var
).grid(row=0, column=1, sticky="w", pady=4)

tk.Checkbutton(
    checkbox_grid, text="Device ID", variable=root.include_device_id_var
).grid(row=1, column=0, sticky="w", padx=(0, 30), pady=4)

tk.Checkbutton(checkbox_grid, text="Timestamp", variable=root.include_time_var).grid(
    row=1, column=1, sticky="w", pady=4
)

tk.Checkbutton(checkbox_grid, text="Location", variable=root.include_location_var).grid(
    row=2, column=0, sticky="w", padx=(0, 30), pady=4
)

# ------------------------------
# Interval controls
# ------------------------------
interval_frame = make_section(left_col, "Interval Settings", 1)

root.default_interval_var = tk.IntVar(value=1)

tk.Label(interval_frame, text="Interval (minutes):").grid(
    row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
)

interval_row = tk.Frame(interval_frame)
interval_row.grid(row=1, column=0, sticky="w")

default_interval_cb = tk.Checkbutton(
    interval_row,
    text="Use default interval (1)",
    variable=root.default_interval_var,
)
default_interval_cb.grid(row=0, column=0, sticky="w")

root.interval_var = tk.IntVar(
    value=int(read_user_config_value("INTERVAL_BETWEEN_REPEATS_MINUTES"))
)

root.interval_field = tk.Spinbox(
    interval_row,
    from_=1,
    to=1440,
    width=SMALL_ENTRY_WIDTH,
    textvariable=root.interval_var,
)
root.interval_field.grid(row=0, column=1, padx=(16, 0), sticky="w")

# -------------------------------
# Delay initial timestamp setting
# -------------------------------
delay_frame = make_section(right_col, "Initial Delay Settings", 1)

root.default_delay_var = tk.IntVar(value=0)

tk.Label(delay_frame, text="Initial Delay (minutes):").grid(
    row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
)

delay_row = tk.Frame(delay_frame)
delay_row.grid(row=1, column=0, sticky="w")

default_delay_cb = tk.Checkbutton(
    delay_row,
    text="Start at a specific minute (0-59)",
    variable=root.default_delay_var,
)
default_delay_cb.grid(row=0, column=0, sticky="w")

root.delay_var = tk.IntVar(value=int(read_user_config_value("STARTING_MINUTE")))

root.delay_field = tk.Spinbox(
    delay_row,
    from_=0,
    to=59,
    width=SMALL_ENTRY_WIDTH,
    textvariable=root.delay_var,
)
root.delay_field.grid(row=0, column=1, padx=(16, 0), sticky="w")

# ------------------------------
# Transmission Settings
# ------------------------------
root.transmission_var = tk.StringVar(value="cable")

transmission_frame = make_section(left_col, "Transmission Settings", 2)

tk.Radiobutton(
    transmission_frame,
    text="Use Cable Transmission",
    variable=root.transmission_var,
    value="cable",
).grid(row=0, column=0, sticky="w", pady=4)

tk.Radiobutton(
    transmission_frame,
    text="Use Speaker Transmission",
    variable=root.transmission_var,
    value="speaker",
).grid(row=1, column=0, sticky="w", pady=4)

# ------------------------------
# Frequency Settings
# ------------------------------
frequency_frame = make_section(right_col, "FSK Parameters", 2)

tk.Label(
    frequency_frame,
    text="Select a pair of frequencies to use for FSK modulation (1000–24000 Hz):",
    fg="gray",
    wraplength=430,
    justify="left",
).grid(row=0, column=0, sticky="w", pady=(0, 10))

root.frequency_low_var = tk.IntVar(
    value=int(read_user_config_value("FSK_LOWER_FREQUENCY"))
)
root.frequency_high_var = tk.IntVar(
    value=int(read_user_config_value("FSK_HIGHER_FREQUENCY"))
)

freq_row = tk.Frame(frequency_frame)
freq_row.grid(row=1, column=0, sticky="w")

tk.Label(freq_row, text="Low (Hz):").grid(row=0, column=0, sticky="w")
root.frequency_low_field = tk.Entry(
    freq_row,
    width=SMALL_ENTRY_WIDTH,
    textvariable=root.frequency_low_var,
)
root.frequency_low_field.grid(row=0, column=1, sticky="w", padx=(8, 26))

tk.Label(freq_row, text="High (Hz):").grid(row=0, column=2, sticky="w")
root.frequency_high_field = tk.Entry(
    freq_row,
    width=SMALL_ENTRY_WIDTH,
    textvariable=root.frequency_high_var,
)
root.frequency_high_field.grid(row=0, column=3, sticky="w", padx=(8, 0))

# ------------------------------
# Attenuation settings
# ------------------------------
attunent_frame = make_section(left_col, "Attenuation Settings", 3)

root.attenuation_var = tk.IntVar(
    value=int(read_user_config_value("SIGNAL_ATTENUATION"))
)

tk.Label(
    attunent_frame,
    text="Enter a value between 100 % (loudest) and 0 % (most attenuated)",
    fg="gray",
    wraplength=430,
    justify="left",
).grid(row=0, column=0, sticky="w", pady=(0, 8))

attenuation_input_frame = tk.Frame(attunent_frame)
attenuation_input_frame.grid(row=1, column=0, sticky="w")

tk.Label(attenuation_input_frame, text="Attenuation:").grid(row=0, column=0, sticky="w")
root.attenuation_field = tk.Spinbox(
    attenuation_input_frame,
    from_=0,
    to=100,
    width=SMALL_ENTRY_WIDTH,
    textvariable=root.attenuation_var,
)
root.attenuation_field.grid(row=0, column=1, padx=(10, 0), sticky="w")

root.attenuation_db_label = tk.Label(
    attunent_frame,
    text="",
    fg="gray",
    justify="left",
)
root.attenuation_db_label.grid(row=2, column=0, sticky="w", pady=(8, 0))


def calculate_attenuation_db(x: int) -> str:
    try:
        value = 0.2 * float(x) / 100.0
        if value <= 0:
            return "-∞ dB"
        db = 20.0 * math.log10(value)
        return f"{db:.2f} dB"
    except Exception:
        return "Invalid value"


def update_attenuation_db_label(*args):
    try:
        x = int(root.attenuation_var.get())
    except Exception:
        root.attenuation_db_label.config(text="dB value: Invalid value")
        return

    db_text = calculate_attenuation_db(x)
    root.attenuation_db_label.config(text=f"dB value: {db_text}")


root.attenuation_var.trace_add("write", update_attenuation_db_label)
root.attenuation_field.bind("<KeyRelease>", update_attenuation_db_label)
root.attenuation_field.bind("<FocusOut>", update_attenuation_db_label)
update_attenuation_db_label()

# ------------------------------
# Debug settings
# ------------------------------
show_log_frame = make_section(right_col, "Debug Settings", 3)

show_log_var = tk.IntVar(value=0)
tk.Checkbutton(show_log_frame, text="Show build log", variable=show_log_var).grid(
    row=0, column=0, sticky="w", pady=4
)

# ------------------------------
# Frequency helpers
# ------------------------------
FS_HZ = 960000
MIN_BIT_US = 3000
FREQ_MIN = 1000
FREQ_MAX = 24000


def rounded_min_bit_samples(fs: int) -> int:
    return ((fs * MIN_BIT_US) + 500000) // 1000000


def quantized_freq_from_samples(fs: int, samples_per_period: int) -> int:
    if samples_per_period <= 0:
        return 0
    return int(math.floor(fs / samples_per_period))


def period_count_from_samples(min_bit_samples: int, samples_per_period: int) -> int:
    return int(round(min_bit_samples / samples_per_period))


def freq_diff_u16(a: int, b: int) -> int:
    return abs(a - b)


def min_required_diff_hz(lower_freq: int) -> int:
    return 300 + (400000 // lower_freq)


def quantized_params(freq: float, fs: int = FS_HZ):
    samples_per_period = max(1, int(math.floor(fs / freq)))
    min_bit_samples = rounded_min_bit_samples(fs)
    period_count = period_count_from_samples(min_bit_samples, samples_per_period)
    quantized_freq = quantized_freq_from_samples(fs, samples_per_period)
    total_samples = samples_per_period * period_count
    return quantized_freq, samples_per_period, period_count, total_samples


def adjust_low_frequency_to_valid(low_freq: float, high_freq: float):
    """
    Keep high fixed.
    Adjust only low downward until:
      - low/high period counts differ
      - spacing satisfies min_required_diff_hz(lower_freq)
    """
    min_bit_samples = rounded_min_bit_samples(FS_HZ)

    low_n = max(1, int(math.floor(FS_HZ / low_freq)))
    high_n = max(1, int(math.floor(FS_HZ / high_freq)))

    high_q = quantized_freq_from_samples(FS_HZ, high_n)
    high_p = period_count_from_samples(min_bit_samples, high_n)

    while True:
        low_q = quantized_freq_from_samples(FS_HZ, low_n)
        low_p = period_count_from_samples(min_bit_samples, low_n)

        same_periods = low_p == high_p
        enough_diff = freq_diff_u16(high_q, low_q) >= min_required_diff_hz(low_q)

        if (not same_periods) and enough_diff and low_q < high_q:
            break

        # lower the low frequency
        candidate_low_n = low_n + 1
        candidate_low_q = quantized_freq_from_samples(FS_HZ, candidate_low_n)

        if candidate_low_q < FREQ_MIN:
            break

        low_n = candidate_low_n

    low_q = quantized_freq_from_samples(FS_HZ, low_n)
    low_p = period_count_from_samples(min_bit_samples, low_n)
    total_samples = low_n * low_p

    return low_q, low_n, low_p, total_samples


def adjust_high_frequency_to_valid(low_freq: float, high_freq: float):
    """
    Keep low fixed.
    Adjust only high upward until:
      - low/high period counts differ
      - spacing satisfies min_required_diff_hz(lower_freq)
    """
    min_bit_samples = rounded_min_bit_samples(FS_HZ)

    low_n = max(1, int(math.floor(FS_HZ / low_freq)))
    high_n = max(1, int(math.floor(FS_HZ / high_freq)))

    low_q = quantized_freq_from_samples(FS_HZ, low_n)
    low_p = period_count_from_samples(min_bit_samples, low_n)

    while True:
        high_q = quantized_freq_from_samples(FS_HZ, high_n)
        high_p = period_count_from_samples(min_bit_samples, high_n)

        same_periods = low_p == high_p
        enough_diff = freq_diff_u16(high_q, low_q) >= min_required_diff_hz(low_q)

        if (not same_periods) and enough_diff and high_q > low_q:
            break

        # raise the high frequency
        if high_n <= 1:
            break

        candidate_high_n = high_n - 1
        candidate_high_q = quantized_freq_from_samples(FS_HZ, candidate_high_n)

        if candidate_high_q > FREQ_MAX:
            break

        high_n = candidate_high_n

    high_q = quantized_freq_from_samples(FS_HZ, high_n)
    high_p = period_count_from_samples(min_bit_samples, high_n)
    total_samples = high_n * high_p

    return high_q, high_n, high_p, total_samples


def update_low_frequency():
    try:
        low = float(root.frequency_low_var.get())
        high = float(root.frequency_high_var.get())
    except Exception:
        return

    low = max(FREQ_MIN, min(FREQ_MAX, low))
    high = max(FREQ_MIN, min(FREQ_MAX, high))

    if low >= high:
        low = high - 1

    if low < FREQ_MIN:
        low = FREQ_MIN

    new_low, _, _, _ = adjust_low_frequency_to_valid(low, high)
    root.frequency_low_var.set(int(new_low))


def update_high_frequency():
    try:
        low = float(root.frequency_low_var.get())
        high = float(root.frequency_high_var.get())
    except Exception:
        return

    low = max(FREQ_MIN, min(FREQ_MAX, low))
    high = max(FREQ_MIN, min(FREQ_MAX, high))

    if high <= low:
        high = low + 1

    if high > FREQ_MAX:
        high = FREQ_MAX

    new_high, _, _, _ = adjust_high_frequency_to_valid(low, high)
    root.frequency_high_var.set(int(new_high))


root.frequency_low_field.bind("<FocusOut>", lambda e: update_low_frequency())
root.frequency_low_field.bind("<Return>", lambda e: update_low_frequency())

root.frequency_high_field.bind("<FocusOut>", lambda e: update_high_frequency())
root.frequency_high_field.bind("<Return>", lambda e: update_high_frequency())

update_low_frequency()
update_high_frequency()


# ------------------------------
# Field enabling/disabling logic
# ------------------------------
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
# Global click to defocus entries
# ------------------------------
def defocus_all(event):
    w = event.widget
    if isinstance(w, (tk.Entry, tk.Spinbox, ttk.Entry)):
        return
    root.focus_set()


root.bind_all("<Button-1>", defocus_all, add="+")


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

    if not is_field_valid(root.frequency_low_field, int, 1000, 24000):
        invalid_fields.append(
            "Lower Frequency must be an integer between 1000 and 24000 Hz"
        )

    if not is_field_valid(root.frequency_high_field, int, 1000, 24000):
        invalid_fields.append(
            "Higher Frequency must be an integer between 1000 and 24000 Hz"
        )

    if is_field_valid(root.frequency_low_field, int, 1000, 24000) and is_field_valid(
        root.frequency_high_field, int, 1000, 24000
    ):
        low = int(root.frequency_low_field.get().strip())
        high = int(root.frequency_high_field.get().strip())
        if low >= high:
            invalid_fields.append("Lower Frequency must be less than Higher Frequency")

        q_low_from_low, _, low_p, _ = adjust_low_frequency_to_valid(low, high)
        q_high_from_high, _, high_p, _ = adjust_high_frequency_to_valid(low, high)

        min_diff_low = min_required_diff_hz(q_low_from_low)
        actual_diff_low = freq_diff_u16(high, q_low_from_low)

        min_diff_high = min_required_diff_hz(q_high_from_high)
        actual_diff_high = freq_diff_u16(q_high_from_high, low)

        if q_low_from_low >= high:
            invalid_fields.append("Lower Frequency becomes invalid after adjustment")

        if q_high_from_high <= low:
            invalid_fields.append("Higher Frequency becomes invalid after adjustment")

        if actual_diff_low < min_diff_low and actual_diff_high < min_diff_high:
            invalid_fields.append(
                "Frequency difference is too small after quantization/adjustment"
            )

    if not is_field_valid(root.attenuation_field, int, 0, 100):
        invalid_fields.append("Attenuation must be an integer between 0 and 100")

    if invalid_fields:
        formatted = "\n".join(f"• {msg}" for msg in invalid_fields)
        messagebox.showwarning(
            "Invalid Input",
            f"⚠️ Please correct the following fields:\n\n{formatted}",
        )
        return False

    return True


# ------------------------------
# Build button, progress bar, and status
# ------------------------------
build_section = tk.Frame(main_frame)
build_section.grid(row=2, column=0, columnspan=2, pady=(10, 0))

build_btn = tk.Button(
    build_section,
    text="Build & Flash",
    width=24,
    font=("Arial", 12, "bold"),
    command=lambda: start_dual_flash_thread(),
)
build_btn.pack(pady=(0, 10))

ttk.Separator(build_section, orient="horizontal").pack(fill="x", padx=8, pady=(0, 10))

progress_bar = ttk.Progressbar(
    build_section,
    mode="indeterminate",
    length=900,
)
progress_bar.pack(fill="x", padx=8)

status_label = tk.Label(
    build_section,
    text="Idle",
    font=("Arial", 12, "italic"),
)
status_label.pack(pady=(10, 0))


# ------------------------------
# Background build logic
# ------------------------------
def start_dual_flash_thread():
    if not validate_all_fields():
        return

    build_btn.config(state="disabled")
    status_label.config(text="Building...")
    progress_bar.start(12)

    threading.Thread(target=dual_build_flash_call, daemon=True).start()


def dual_build_flash_call():
    try:
        if not dual_build_flash(
            root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET
        ):
            root.after(0, lambda: progress_bar.stop())
            root.after(0, lambda: status_label.config(text="❌ Build failed"))
            return

        root.after(
            0,
            lambda: [
                progress_bar.stop(),
                status_label.config(text="✅ Build & Flash completed successfully!"),
                messagebox.showinfo(
                    "Build & Flash", "Build and Flash completed successfully!"
                ),
            ],
        )
    except Exception as e:
        root.after(
            0,
            lambda: [
                progress_bar.stop(),
                status_label.config(text="❌ Build failed"),
                messagebox.showerror("Error", f"❌ {e}"),
            ],
        )
    finally:
        root.after(0, lambda: build_btn.config(state="normal"))
        root.after(4000, lambda: status_label.config(text="Idle"))


root.mainloop()
