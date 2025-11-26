import os
import sys
import tkinter as tk
from tkinter import messagebox
import tempfile

from scripts.paths import PROJECT_SRC


def ensure_user_config(action, safe_log=None):
    config_path = os.path.join(PROJECT_SRC, "Core", "Inc", "user_config.h")
    if not action:
        active_src = os.path.join(tempfile.gettempdir(), "stm32_src")
        config_path = os.path.join(active_src, "Core", "Inc", "user_config.h")

    if not os.path.exists(config_path):
        if safe_log:
            safe_log(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)
    else:
        if safe_log:
            safe_log(f"[DEBUG] Using config file: {config_path}")
    return config_path


def read_user_config_value(var_name, safe_log=None):
    config_path = ensure_user_config(True, safe_log)
    try:
        with open(config_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line.startswith(f"#define {var_name}"):
                    continue
                # Split on spaces and quotes
                parts = line.split(maxsplit=2)
                if len(parts) < 3:
                    return "-"
                value = parts[2].strip()

                if "//" in value:
                    value = value.split("//")[0].strip()

                # Handle quoted strings
                if value.startswith('"') and value.endswith('"'):
                    return value.strip('"')

                # Handle numeric values (int or float)
                try:
                    if "." in value:
                        return float(value)
                    return int(value)
                except ValueError:
                    return value
    except Exception:
        if safe_log:
            safe_log(f"[ERROR] Failed to read config file: {config_path}")
        pass
    return "-"  # Default value if not found


def read_current_time():
    from datetime import datetime

    current = datetime.now()

    return (
        current.year,
        current.month,
        current.day,
        current.weekday() + 1,
        current.hour,
        current.minute,
        current.second,
    )


def read_frequency_pairs():
    config_path = ensure_user_config(True)
    pairs = []
    try:
        with open(config_path, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("// 1 = "):
                    try:
                        parts = line.split("= ")[1].split("and")
                        f0 = float(parts[0].strip())
                        f1 = float(parts[1].strip())
                        pairs.append((f0, f1))
                    except (IndexError, ValueError):
                        continue
    except Exception:
        pass
    return pairs  # List of tuples (f0, f1)


def change_user_config(root, set_initial_time, safe_log=None):

    config_path = ensure_user_config(False, safe_log)

    if not os.path.exists(config_path):
        if safe_log:
            safe_log(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)
    else:
        if safe_log:
            safe_log(f"[DEBUG] Using config file: {config_path}")

    try:
        # Read values from GUI fields
        user_string = root.user_string_field.get().strip()
        device_id = root.device_id_field.get().strip()
        location = root.location_field.get().strip()
        frequency_pair = root.frequency_pair_var.get().strip()
        delay_minutes = root.delay_field.get().strip()
        interval_minutes = root.interval_field.get().strip()

        include_user_string = root.include_user_string_var.get()
        include_device_id = root.include_device_id_var.get()
        include_location = root.include_location_var.get()
        include_temperature = root.include_temperature_var.get()
        include_time = root.include_time_var.get()

        enable_delayed_start = root.default_delay_var.get()
        use_default_interval = root.default_interval_var.get()

        use_cable_driven_transmission = root.use_cable_transmission.get()
        use_speaker_driven_transmission = root.use_speaker_transmission.get()
    except Exception as e:
        safe_log(f"[ERROR] Failed to read GUI fields: {str(e)}")
        return False

    # Get the current time
    current_time = read_current_time()

    # Variables that are integer-style (no quotes)
    int_vars = {
        "DEVICE_ID",
        "TEMPERATURE",
        "INITIAL_YEAR",
        "INITIAL_MONTH",
        "INITIAL_DOW",
        "INITIAL_DOM",
        "INITIAL_HOUR",
        "INITIAL_MIN",
        "INITIAL_SEC",
        "FSK_FREQUENCY_PAIR",
        "STARTING_MINUTE",
        "INTERVAL_BETWEEN_REPEATS_MINUTES",
    }

    # Values to apply
    variables = {
        "USER_STRING": user_string,
        "DEVICE_ID": device_id,
        "LOCATION": location,
        "INCLUDE_USER_STRING": include_user_string,
        "INCLUDE_DEVICE_ID": include_device_id,
        "INCLUDE_LOCATION": include_location,
        "INCLUDE_TEMPERATURE": include_temperature,
        "INCLUDE_TIME": include_time,
        "SET_INITIAL_TIME": set_initial_time,
        "INITIAL_YEAR": current_time[0],
        "INITIAL_MONTH": current_time[1],
        "INITIAL_DOM": current_time[2],
        "INITIAL_DOW": current_time[3],
        "INITIAL_HOUR": current_time[4],
        "INITIAL_MIN": current_time[5],
        "INITIAL_SEC": current_time[6],
        "FSK_FREQUENCY_PAIR": frequency_pair,
        "ENABLE_DELAYED_START": enable_delayed_start,
        "STARTING_MINUTE": delay_minutes,
        "USE_DEFAULT_INTERVAL_BETWEEN_REPEATS": use_default_interval,
        "INTERVAL_BETWEEN_REPEATS_MINUTES": interval_minutes,
        "USE_CABLE_DRIVEN_TRANSMISSION": use_cable_driven_transmission,
        "USE_SPEAKER_DRIVEN_TRANSMISSION": use_speaker_driven_transmission,
    }

    if not user_string:
        safe_log("[ERROR] User string is empty.")
        messagebox.showerror("Error", "Please enter a valid string.")
        return False

    try:
        with open(config_path, "r") as file:
            lines = file.readlines()

        for i, line in enumerate(lines):
            for var, val in variables.items():
                # Strip leading spaces before comparing
                if f"#define {var}" in line:
                    if (
                        "INCLUDE" in var
                        or var == "SET_INITIAL_TIME"
                        or var == "USE_DEFAULT_INTERVAL_BETWEEN_REPEATS"
                        or var == "ENABLE_DELAYED_START"
                        or var == "USE_CABLE_DRIVEN_TRANSMISSION"
                        or var == "USE_SPEAKER_DRIVEN_TRANSMISSION"
                    ):
                        val_str = "true" if val else "false"
                        lines[i] = f"#define {var} {val_str}\n"
                    elif var in int_vars:
                        lines[i] = f"#define {var} {val}\n"
                    else:
                        val_clean = str(val).replace('"', "")
                        lines[i] = f'#define {var} "{val_clean}"\n'

        # Append any missing defines at the end
        for var, val in variables.items():
            if not any(f"#define {var}" in line for line in lines):
                if (
                    "INCLUDE" in var
                    or var == "SET_INITIAL_TIME"
                    or var == "FSK_FREQUENCY_PAIR"
                ):
                    val_str = "true" if val else "false"
                    lines.append(f"#define {var} {val_str}\n")
                elif var in int_vars:
                    lines.append(f"#define {var} {val}\n")
                else:
                    val_clean = str(val).replace('"', "")
                    lines.append(f'#define {var} "{val_clean}"\n')

        if not any("#endif" in l for l in lines):
            lines.append("\n#endif // USER_CONFIG_H\n")

        with open(config_path, "w") as file:
            file.writelines(lines)

        return True

    except Exception as e:
        safe_log(f"[ERROR] Failed to update config file: {str(e)}")
        messagebox.showerror("Error", f"Failed to update config: {str(e)}")
        return False
