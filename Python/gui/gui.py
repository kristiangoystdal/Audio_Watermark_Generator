import os
import sys
import subprocess
import shutil
import tempfile
import tkinter as tk
from tkinter import messagebox

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
PROJECT_SRC = os.path.join(BASE, "STM32")
BUILD_DIR = os.path.join(tempfile.gettempdir(), "stm32_build")
TOOLS_DIR = os.path.join(BASE, "tools")

TOOLCHAIN = os.path.join(TOOLS_DIR, "arm-none-eabi-gcc")
CMAKE = os.path.join(TOOLS_DIR, "cmake", "4.1.2", "bin", "cmake")
NINJA = os.path.join(TOOLS_DIR, "ninja", "ninja")
OPENOCD = os.path.join(TOOLS_DIR, "openocd", "bin", "openocd")
TOOLCHAIN_FILE = os.path.join(TOOLS_DIR, "cmake", "arm-gcc-toolchain.cmake")

OPENOCD_INTERFACE = os.path.join(
    TOOLS_DIR, "openocd", "share", "openocd", "scripts", "interface", "stlink.cfg"
)
OPENOCD_TARGET = os.path.join(
    TOOLS_DIR, "openocd", "share", "openocd", "scripts", "target", "stm32g4x.cfg"
)


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
def clean_build():
    if os.path.exists(BUILD_DIR):
        shutil.rmtree(BUILD_DIR)


def find_elf():
    for root, _, files in os.walk(BUILD_DIR):
        for f in files:
            if f.endswith(".elf") or f == "Audio_Watermark_Generator":
                return os.path.join(root, f)
    return None


def ensure_user_config():
    """Ensure user_config.h exists, create default if missing."""
    config_path = os.path.join(PROJECT_SRC, "Core", "Inc", "user_config.h")
    if not os.path.exists(config_path):
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, "w") as f:
            f.write('#define USER_STRING "Default"\n')
    return config_path


def read_user_string():
    """Read current USER_STRING from user_config.h."""
    config_path = ensure_user_config()
    try:
        with open(config_path, "r") as f:
            for line in f:
                if line.strip().startswith("#define USER_STRING"):
                    parts = line.split('"')
                    if len(parts) >= 2:
                        return parts[1]
    except Exception:
        pass
    return "Hello World"  # fallback default


def change_user_config():
    config_path = ensure_user_config()
    user_string = entry.get().strip()
    if not user_string:
        messagebox.showerror("Error", "Please enter a valid string.")
        return False
    try:
        with open(config_path, "r") as file:
            lines = file.readlines()
        updated = False
        for i, line in enumerate(lines):
            if line.startswith("#define USER_STRING"):
                lines[i] = f'#define USER_STRING "{user_string}"\n'
                updated = True
        if not updated:
            lines.append(f'#define USER_STRING "{user_string}"\n')
        with open(config_path, "w") as file:
            file.writelines(lines)
        return True
    except Exception as e:
        messagebox.showerror("Error", f"Failed to update config: {str(e)}")
        return False


# ---------------------------------------------------------
# Build + Flash combined
# ---------------------------------------------------------
def build_and_flash():
    clean_build()
    os.makedirs(BUILD_DIR, exist_ok=True)

    if not change_user_config():
        return

    try:
        subprocess.check_call(
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
            ]
        )

        subprocess.check_call([CMAKE, "--build", BUILD_DIR])

        elf_file = find_elf()
        if not elf_file:
            messagebox.showerror("Flash", "❌ No ELF file found after build.")
            return

        subprocess.check_call(
            [
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
        )

        messagebox.showinfo("Success", "✅ Build and Flash completed successfully!")

    except subprocess.CalledProcessError as e:
        messagebox.showerror("Error", f"❌ Failed during build/flash: {str(e)}")


# ---------------------------------------------------------
# GUI
# ---------------------------------------------------------
root = tk.Tk()
root.title("STM32 Builder & Flasher")

frame = tk.Frame(root, padx=20, pady=20)
frame.pack()

tk.Label(frame, text="STM32 Build & Flash Tool", font=("Arial", 14, "bold")).pack(
    pady=10
)

entry = tk.Entry(frame)
entry.pack(pady=5)
# Pre-fill with whatever is in user_config.h
entry.insert(0, read_user_string())

tk.Button(frame, text="🔨⚡ Build & Flash", width=25, command=build_and_flash).pack(
    pady=10
)

root.mainloop()
