import os
import sys
import tkinter as tk
from tkinter import messagebox

from scripts.paths import PROJECT_SRC


def ensure_user_config():
    config_path = os.path.join(PROJECT_SRC, "Core", "Inc", "user_config.h")
    if not os.path.exists(config_path):
        messagebox.showerror("Error", f"Config file not found: {config_path}")
        sys.exit(1)
    return config_path


def read_user_config_value(var_name):
    """Read a #define value from user_config.h (supports both quoted and numeric values)."""
    config_path = ensure_user_config()
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


def change_user_config(root, set_initial_time):

    config_path = ensure_user_config()

    # Read values from GUI fields
    user_string = root.user_string_field.get().strip()
    device_id = root.device_id_field.get().strip()
    location = root.location_field.get().strip()
    temperature = root.temperature_field.get().strip()

    include_user_string = root.include_user_string_var.get()
    include_device_id = root.include_device_id_var.get()
    include_location = root.include_location_var.get()
    include_temperature = root.include_temperature_var.get()
    include_time = root.include_time_var.get()

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
    }

    # Values to apply
    variables = {
        "USER_STRING": user_string,
        "DEVICE_ID": device_id,
        "LOCATION": location,
        "TEMPERATURE": temperature,
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
    }

    if not user_string:
        messagebox.showerror("Error", "Please enter a valid string.")
        return False

    try:
        with open(config_path, "r") as file:
            lines = file.readlines()

        updated = False
        for i, line in enumerate(lines):
            for var, val in variables.items():
                # Strip leading spaces before comparing
                if f"#define {var}" in line:
                    updated = True
                    if "INCLUDE" in var or var == "SET_INITIAL_TIME":
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
                if "INCLUDE" in var or var == "SET_INITIAL_TIME":
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

        messagebox.showinfo("Success", "✅ user_config.h updated successfully.")
        return True

    except Exception as e:
        messagebox.showerror("Error", f"Failed to update config: {str(e)}")
        return False
