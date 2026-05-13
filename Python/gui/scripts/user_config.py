from datetime import datetime, UTC
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

    try:
        user_string = root.vars["user_string"].get().strip()
        device_id = root.vars["device_id"].get().strip()
        location = root.vars["location"].get().strip()

        starting_hour = str(root.vars["starting_hour"].get()).strip()
        starting_minute = str(root.vars["starting_minute"].get()).strip()
        end_hour = str(root.vars["end_hour"].get()).strip()
        end_minute = str(root.vars["end_minute"].get()).strip()

        run_minutes = str(root.vars["run_minutes"].get()).strip()
        sleep_minutes = str(root.vars["sleep_minutes"].get()).strip()

        include_user_string = root.vars["include_user_string"].get()
        include_device_id = root.vars["include_device_id"].get()
        include_location = root.vars["include_location"].get()
        include_temperature = root.vars["include_temperature"].get()
        include_time = root.vars["include_time"].get()

        transmission = root.vars["transmission"].get()
        use_cable_transmission = transmission == "cable"
        use_speaker_transmission = transmission == "speaker"

        frequency_lower = root.vars["frequency_low"].get()
        frequency_higher = root.vars["frequency_high"].get()

        attenuation = root.vars["attenuation"].get()
        rs_error_correction_symbols = root.vars["ecc_level"].get()
        ecc_enabled = root.vars["ecc_enabled"].get()

        operation_mode = root.vars["operation_mode"].get()

    except Exception as e:
        if safe_log:
            safe_log(f"[ERROR] Failed to read GUI fields: {str(e)}")
        return False

    if not user_string:
        if safe_log:
            safe_log("[ERROR] User string is empty.")
        messagebox.showerror("Error", "Please enter a valid string.")
        return False

    def to_int(name: str, v, min_v=None, max_v=None):
        try:
            iv = int(str(v).strip())
        except Exception:
            raise ValueError(f"{name} must be an integer (got {v!r})")
        if min_v is not None and iv < min_v:
            iv = min_v
        if max_v is not None and iv > max_v:
            iv = max_v
        return iv

    def to_str(v):
        return str(v).strip().replace('"', "")

    current_time = read_current_time()

    try:
        device_id_i = to_int("DEVICE_ID", device_id, 0, 65535)
        starting_hour_i = to_int("STARTING_HOUR", starting_hour, 0, 23)
        starting_minute_i = to_int("STARTING_MINUTE", starting_minute, 0, 59)
        end_hour_i = to_int("END_HOUR", end_hour, 0, 23)
        end_minute_i = to_int("END_MINUTE", end_minute, 0, 59)
        run_minutes_i = to_int("RUN_MINUTES", run_minutes, 0, 1440)
        sleep_minutes_i = to_int("SLEEP_MINUTES", sleep_minutes, 0, 1440)

        fsk_low_i = to_int("FSK_LOWER_FREQUENCY", frequency_lower, 1, 50000)
        fsk_high_i = to_int("FSK_HIGHER_FREQUENCY", frequency_higher, 1, 50000)

        attenuation_i = to_int("SIGNAL_ATTENUATION", attenuation, 0, 120)
        rs_error_correction_symbols_i = to_int(
            "RS_ERROR_CORRECTION_SYMBOLS", rs_error_correction_symbols, 0, 100
        )

    except ValueError as e:
        if safe_log:
            safe_log(f"[ERROR] {e}")
        messagebox.showerror("Error", str(e))
        return False

    bool_vars = {
        "INCLUDE_USER_STRING",
        "INCLUDE_DEVICE_ID",
        "INCLUDE_LOCATION",
        "INCLUDE_TEMPERATURE",
        "INCLUDE_TIME",
        "SET_INITIAL_TIME",
        "USE_CABLE_TRANSMISSION",
        "USE_SPEAKER_TRANSMISSION",
        "USE_REED_SOLOMON_ERROR_CORRECTION",
    }

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
        "STARTING_HOUR",
        "STARTING_MINUTE",
        "END_HOUR",
        "END_MINUTE",
        "RUN_MINUTES",
        "SLEEP_MINUTES",
        "FSK_LOWER_FREQUENCY",
        "FSK_HIGHER_FREQUENCY",
        "SIGNAL_ATTENUATION",
        "RS_ERROR_CORRECTION_SYMBOLS",
        "OPERATION_MODE",
    }

    variables = {
        "USER_STRING": to_str(user_string),
        "DEVICE_ID": device_id_i,
        "LOCATION": to_str(location),
        "INCLUDE_USER_STRING": bool(include_user_string),
        "INCLUDE_DEVICE_ID": bool(include_device_id),
        "INCLUDE_LOCATION": bool(include_location),
        "INCLUDE_TEMPERATURE": bool(include_temperature),
        "INCLUDE_TIME": bool(include_time),
        "SET_INITIAL_TIME": bool(set_initial_time),
        "INITIAL_YEAR": current_time[0],
        "INITIAL_MONTH": current_time[1],
        "INITIAL_DOM": current_time[2],
        "INITIAL_DOW": current_time[3],
        "INITIAL_HOUR": current_time[4],
        "INITIAL_MIN": current_time[5],
        "INITIAL_SEC": current_time[6],
        "STARTING_HOUR": starting_hour_i,
        "STARTING_MINUTE": starting_minute_i,
        "END_HOUR": end_hour_i,
        "END_MINUTE": end_minute_i,
        "RUN_MINUTES": run_minutes_i,
        "SLEEP_MINUTES": sleep_minutes_i,
        "FSK_LOWER_FREQUENCY": fsk_low_i,
        "FSK_HIGHER_FREQUENCY": fsk_high_i,
        "USE_CABLE_TRANSMISSION": bool(use_cable_transmission),
        "USE_SPEAKER_TRANSMISSION": bool(use_speaker_transmission),
        "USE_REED_SOLOMON_ERROR_CORRECTION": bool(ecc_enabled),
        "SIGNAL_ATTENUATION": attenuation_i,
        "RS_ERROR_CORRECTION_SYMBOLS": rs_error_correction_symbols_i,
        "OPERATION_MODE": operation_mode,
    }

    def format_define(var: str, val):
        if var in bool_vars:
            return f"#define {var} {'true' if val else 'false'}\n"
        if var in int_vars:
            return f"#define {var} {int(val)}\n"
        return f'#define {var} "{to_str(val)}"\n'

    try:
        with open(config_path, "r") as file:
            lines = file.readlines()

        for i, line in enumerate(lines):
            stripped = line.lstrip()
            if not stripped.startswith("#define "):
                continue

            for var, val in variables.items():
                if stripped.startswith(f"#define {var}"):
                    lines[i] = format_define(var, val)
                    break

        for var, val in variables.items():
            if not any(l.lstrip().startswith(f"#define {var}") for l in lines):
                lines.append(format_define(var, val))

        if not any(l.strip().startswith("#endif") for l in lines):
            lines.append("\n#endif // USER_CONFIG_H\n")

        with open(config_path, "w") as file:
            file.writelines(lines)

        return True

    except Exception as e:
        if safe_log:
            safe_log(f"[ERROR] Failed to update config file: {str(e)}")
        messagebox.showerror("Error", f"Failed to update config: {str(e)}")
        return False
