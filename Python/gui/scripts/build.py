import os
import shutil
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox
import tkinter.scrolledtext as st

# Import paths
from scripts.paths import *
from scripts.user_config import *



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


def build_and_flash(root, show_log_var, OPENOCD_INTERFACE, OPENOCD_TARGET):
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


def dual_build_flash(root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET):
    global set_initial_time
    build_btn.config(state="disabled")
    root.update_idletasks()

    # Change user config and set the clock
    set_initial_time = 1
    if not change_user_config(root, set_initial_time):
        build_btn.config(state="normal")
        return

    # Build and flash with the setting the clock
    build_and_flash(root, show_log_var, OPENOCD_INTERFACE, OPENOCD_TARGET)

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
