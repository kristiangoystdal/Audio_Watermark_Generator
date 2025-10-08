import os
import sys
import subprocess
import shutil
import tempfile
import tkinter as tk
from tkinter import messagebox
import threading
import tkinter.scrolledtext as st


# ---------------------------------------------------------
# Resolve base path (works for source, onefile, onedir macOS app)
# ---------------------------------------------------------
if getattr(sys, "frozen", False):
    if sys.platform == "darwin":
        BASE = os.path.abspath(
            os.path.join(os.path.dirname(sys.executable), "..", "Resources")
        )
    else:
        BASE = sys._MEIPASS
else:
    BASE = os.path.dirname(__file__)

# ---------------------------------------------------------
# Paths
# ---------------------------------------------------------

# ---------------------------------------------------------
# Resolve base path (works for source, onedir, macOS app)
# ---------------------------------------------------------
if getattr(sys, "frozen", False):
    if sys.platform == "darwin":
        # macOS .app layout:
        # STM32Tool.app/
        # └── Contents/
        #     ├── MacOS/STM32Tool  ← executable
        #     └── Resources/STM32  ← project source
        RESOURCES_DIR = os.path.abspath(
            os.path.join(os.path.dirname(sys.executable), "..", "Resources")
        )
        PROJECT_SRC = os.path.join(RESOURCES_DIR, "STM32")
        TOOLS_DIR = os.path.join(RESOURCES_DIR, "tools")

        # Ensure STM32 exists
        if not os.path.exists(PROJECT_SRC):
            messagebox.showerror("Error", f"STM32 folder not found: {PROJECT_SRC}")
            sys.exit(1)

    else:
        # Windows/Linux PyInstaller build
        BASE = sys._MEIPASS
        PROJECT_SRC = os.path.join(BASE, "STM32")
        TOOLS_DIR = os.path.join(BASE, "tools")
else:
    # Running from source
    BASE = os.path.dirname(__file__)
    PROJECT_SRC = os.path.abspath(os.path.join(BASE, "../../STM32"))  # two levels up
    TOOLS_DIR = os.path.join(BASE, "tools")


BUILD_DIR = os.path.join(tempfile.gettempdir(), "stm32_build")

TOOLCHAIN = os.path.join(TOOLS_DIR, "arm-none-eabi-gcc")
# Detect CMake version folder dynamically (handles PyInstaller renaming)
cmake_root = os.path.join(TOOLS_DIR, "cmake")
cmake_versions = [
    d for d in os.listdir(cmake_root) if os.path.isdir(os.path.join(cmake_root, d))
]
if cmake_versions:
    cmake_version_dir = cmake_versions[0]  # take first found
else:
    cmake_version_dir = "4.1.2"

CMAKE = os.path.join(cmake_root, cmake_version_dir, "bin", "cmake")
TOOLCHAIN_FILE = os.path.join(TOOLS_DIR, "cmake", "arm-gcc-toolchain.cmake")
NINJA = os.path.join(TOOLS_DIR, "ninja", "ninja")
OPENOCD = os.path.join(TOOLS_DIR, "openocd", "bin", "openocd")

OPENOCD_INTERFACE = os.path.join(
    TOOLS_DIR, "openocd", "share", "openocd", "scripts", "interface", "stlink.cfg"
)
OPENOCD_TARGET = os.path.join(
    TOOLS_DIR, "openocd", "share", "openocd", "scripts", "target", "stm32g4x.cfg"
)


# ---------------------------------------------------------
# Global variables
# ---------------------------------------------------------
set_initial_time = 1  # Whether to set the clock on the first build/flash


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
def find_elf():
    search_paths = [
        BUILD_DIR,
        os.path.join(PROJECT_SRC, "build"),
        os.path.join(PROJECT_SRC, "build", "Debug"),
        os.path.join(PROJECT_SRC, "build", "Release"),
    ]

    for path in search_paths:
        if not os.path.exists(path):
            continue
        for root, _, files in os.walk(path):
            for f in files:
                if f.endswith(".elf"):
                    return os.path.join(root, f)
    return None


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


def change_user_config():
    global set_initial_time

    config_path = ensure_user_config()

    # Read values from GUI fields
    user_string = user_string_field.get().strip()
    device_id = device_id_field.get().strip()
    location = location_field.get().strip()
    temperature = temperature_field.get().strip()

    include_user_string = include_user_string_var.get()
    include_device_id = include_device_id_var.get()
    include_location = include_location_var.get()
    include_temperature = include_temperature_var.get()
    include_time = include_time_var.get()

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


def run_with_log(cmd, log_text=None):
    """Run a shell command and stream output to log window if enabled."""
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        for line in process.stdout:
            if log_text:
                log_text.insert(tk.END, line)
                log_text.see(tk.END)
                log_text.update_idletasks()
            print(line, end="")  # still print to console for safety
        process.wait()
        return process.returncode
    except Exception as e:
        if log_text:
            log_text.insert(tk.END, f"\n[ERROR] {e}\n")
            log_text.see(tk.END)
        raise


def cleanup_old_builds():
    """Delete any previous stm32_build folders in the system temp dir."""
    import glob
    import shutil
    import tempfile
    import time

    temp_dir = tempfile.gettempdir()
    pattern = os.path.join(temp_dir, "stm32_build*")
    found = glob.glob(pattern)

    if not found:
        print("[DEBUG] No old build folders found.")
        return

    print(f"[DEBUG] Cleaning {len(found)} old stm32_build folders...")

    for folder in found:
        try:
            shutil.rmtree(folder)
            print(f"[DEBUG] Deleted: {folder}")
        except Exception as e:
            print(f"[WARN] Could not delete {folder}: {e}")

    print("[DEBUG] Old stm32_build folders cleaned up.")


# ---------------------------------------------------------
# Build + Flash combined
# ---------------------------------------------------------
def build_and_flash():
    cleanup_old_builds()
    print("[DEBUG] Cleaning build directory to force recompilation...")
    if os.path.exists(BUILD_DIR):
        shutil.rmtree(BUILD_DIR)
    os.makedirs(BUILD_DIR, exist_ok=True)

    # Create log window if enabled
    log_text = None
    if show_log_var.get():
        log_win = tk.Toplevel(root)
        log_win.title("Build Log")
        log_text = st.ScrolledText(log_win, width=100, height=30)
        log_text.pack(padx=10, pady=10)
        log_text.insert(tk.END, "Starting build...\n")

    try:
        # Print debug info for directories
        log_text.insert(tk.END, f"[DEBUG] PROJECT_SRC = {PROJECT_SRC}\n")
        log_text.insert(tk.END, f"[DEBUG] TOOLS_DIR   = {TOOLS_DIR}\n")
        log_text.insert(tk.END, f"[DEBUG] BUILD_DIR   = {BUILD_DIR}\n")
        log_text.insert(tk.END, f"[DEBUG] TOOLCHAIN   = {TOOLCHAIN}\n")
        log_text.insert(tk.END, f"[DEBUG] CMAKE       = {CMAKE}\n")
        log_text.insert(tk.END, f"[DEBUG] NINJA       = {NINJA}\n")
        log_text.insert(tk.END, f"[DEBUG] OPENOCD     = {OPENOCD}\n")
        log_text.insert(tk.END, f"[DEBUG] TOOLCHAIN_FILE = {TOOLCHAIN_FILE}\n")
        log_text.insert(
            tk.END, f"[DEBUG] Using user_config.h at: {ensure_user_config()}\n\n"
        )
        log_text.update_idletasks()

        # Delete CMake cache to ensure no stale paths
        cache_path = os.path.join(BUILD_DIR, "CMakeCache.txt")
        if os.path.exists(cache_path):
            os.remove(cache_path)
            log_text.insert(
                tk.END, "[DEBUG] Removed CMakeCache.txt to force fresh configure.\n"
            )

        # Show the first few lines of user_config.h
        log_text.insert(tk.END, "[DEBUG] --- user_config.h preview ---\n")
        with open(ensure_user_config(), "r") as f:
            for i, line in enumerate(f.readlines()):
                if i >= 20:
                    break
                log_text.insert(tk.END, line)
        log_text.insert(tk.END, "[DEBUG] --- end preview ---\n\n")
        log_text.update_idletasks()

        # Run CMake configure
        log_text.insert(tk.END, "[STEP] Running CMake configure...\n")
        if (
            run_with_log(
                [
                    CMAKE,
                    "-S",
                    PROJECT_SRC,
                    "-B",
                    BUILD_DIR,
                    "-G",
                    "Ninja",
                    f"-DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN_FILE}",
                    f"-DCMAKE_MAKE_PROGRAM={NINJA}",
                    "-DCMAKE_BUILD_TYPE=Release",
                ],
                log_text,
            )
            != 0
        ):
            raise subprocess.CalledProcessError(1, "cmake configure")

        # Run CMake build
        log_text.insert(tk.END, "\n[STEP] Building firmware...\n")
        if run_with_log([CMAKE, "--build", BUILD_DIR], log_text) != 0:
            raise subprocess.CalledProcessError(1, "cmake build")

        # Find ELF
        elf_file = find_elf()
        if not elf_file:
            messagebox.showerror("Flash", "❌ No ELF file found after build.")
            return

        log_text.insert(tk.END, f"\n[DEBUG] Built ELF: {elf_file}\n")

        # Flash firmware
        log_text.insert(tk.END, "\n[STEP] Flashing device...\n")
        flash_cmd = [
            OPENOCD,
            "-s",
            os.path.join(TOOLS_DIR, "openocd", "share", "openocd", "scripts"),
            "-f",
            OPENOCD_INTERFACE,
            "-f",
            OPENOCD_TARGET,
            "-c",
            "transport select hla_swd",
            "-c",
            "adapter speed 100",
            "-c",
            "reset_config srst_only srst_nogate connect_assert_srst",
            "-c",
            f"program {elf_file} verify reset exit",
        ]
        log_text.insert(tk.END, f"[DEBUG] Flash command:\n{' '.join(flash_cmd)}\n\n")

        if run_with_log(flash_cmd, log_text) != 0:
            raise subprocess.CalledProcessError(1, "openocd")

        log_text.insert(tk.END, "\n[SUCCESS] ✅ Build and flash complete!\n")
        messagebox.showinfo("Success", "✅ Build and Flash completed successfully!")

    except subprocess.CalledProcessError as e:
        messagebox.showerror("Error", f"❌ Failed during build/flash: {e}")


def dual_build_flash():
    global set_initial_time
    build_btn.config(state="disabled")
    root.update_idletasks()

    # Change user config and set the clock
    set_initial_time = 1
    if not change_user_config():
        build_btn.config(state="normal")
        return

    # Build and flash with the setting the clock
    build_and_flash()

    root.update_idletasks()

    # # Update the variable to not set the clock
    # set_initial_time = 0
    # if not change_user_config():
    #     build_btn.config(state="normal")
    #     return

    # Build and flash without setting the clock
    # build_and_flash()

    # root.update_idletasks()

    build_btn.config(state="normal")


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


user_string_field = make_labeled_entry(
    frame, "User String", read_user_config_value("USER_STRING"), 1
)
device_id_field = make_labeled_entry(
    frame, "Device ID", int(read_user_config_value("DEVICE_ID")), 2
)
location_field = make_labeled_entry(
    frame, "Location", read_user_config_value("LOCATION"), 3
)
temperature_field = make_labeled_entry(
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

include_user_string_var = tk.IntVar(value=1)
include_device_id_var = tk.IntVar(value=1)
include_location_var = tk.IntVar(value=1)
include_temperature_var = tk.IntVar(value=1)
include_time_var = tk.IntVar(value=1)

# Two-column grid for checkboxes
tk.Checkbutton(
    include_frame, text="User String", variable=include_user_string_var
).grid(row=0, column=0, sticky="w")
tk.Checkbutton(include_frame, text="Device ID", variable=include_device_id_var).grid(
    row=1, column=0, sticky="w"
)
tk.Checkbutton(include_frame, text="Location", variable=include_location_var).grid(
    row=2, column=0, sticky="w"
)
tk.Checkbutton(
    include_frame, text="Temperature", variable=include_temperature_var
).grid(row=0, column=1, sticky="w", padx=20)
tk.Checkbutton(include_frame, text="Timestamp", variable=include_time_var).grid(
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
    frame, text="🔨⚡ Build & Flash", width=25, command=dual_build_flash
)
build_btn.grid(row=9, column=0, columnspan=2, pady=20)

root.mainloop()
