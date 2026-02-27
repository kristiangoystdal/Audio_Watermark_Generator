import os
import shutil
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox
import tkinter.scrolledtext as st
import tempfile
import pathlib
import glob

# Import paths
from scripts.paths import *
from scripts.user_config import *

active_src = PROJECT_SRC  # default path

os.environ["PATH"] = (
    f"{os.path.abspath(os.path.join(os.path.dirname(sys.executable), '..', 'Resources', 'tools', 'ninja'))}:"
    + os.environ.get("PATH", "")
)


def run_with_log(cmd, log_text=None):
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        output = []  # collect all lines for error reporting
        for line in process.stdout:
            output.append(line)
            if log_text:
                log_text.insert(tk.END, line)
                log_text.see(tk.END)
                log_text.update_idletasks()
            print(line, end="")  # also print to console

        process.wait()
        if process.returncode != 0:
            # show captured output when a command fails
            err_msg = (
                f"\n[ERROR] Command failed ({cmd[0]} exited with {process.returncode})\n"
                + "".join(output[-50:])  # last 50 lines of output
            )
            if log_text:
                log_text.insert(tk.END, err_msg + "\n")
                log_text.see(tk.END)
            print(err_msg)
        return process.returncode

    except Exception as e:
        msg = f"\n[EXCEPTION] {e}\n"
        if log_text:
            log_text.insert(tk.END, msg)
            log_text.see(tk.END)
        print(msg)
        raise


def find_elf():
    global active_src

    search_paths = [
        BUILD_DIR,
        os.path.join(BUILD_DIR, "build"),
        os.path.join(BUILD_DIR, "build", "Debug"),
        os.path.join(BUILD_DIR, "build", "Release"),
        active_src,
        os.path.join(active_src, "build"),
        os.path.join(active_src, "build", "Debug"),
        os.path.join(active_src, "build", "Release"),
    ]
    for path in search_paths:
        if not os.path.exists(path):
            continue
        for root, _, files in os.walk(path):
            for f in files:
                if f.endswith(".elf"):
                    return os.path.join(root, f)
    return None


def cleanup_old_builds(safe_log):

    temp_dir = tempfile.gettempdir()
    pattern = os.path.join(temp_dir, "stm32_build*")
    found = glob.glob(pattern)

    if not found:
        safe_log("[DEBUG] No old temp builds found.")
        return

    safe_log(f"[DEBUG] Cleaning {len(found)} old stm32_build folders...")
    for folder in found:
        try:
            shutil.rmtree(folder)
            safe_log(f"[DEBUG] Deleted: {folder}")
        except Exception as e:
            safe_log(f"[WARN] Could not delete {folder}: {e}")
    safe_log("[DEBUG] Old stm32_build folders cleaned up.")


def ensure_writable_copy(safe_log):
    global active_src

    if getattr(sys, "frozen", False) and sys.platform == "darwin":
        temp_copy_dir = os.path.join(tempfile.gettempdir(), "stm32_src")

        app_resources = os.path.abspath(
            os.path.join(os.path.dirname(sys.executable), "..", "Resources", "STM32")
        )

        safe_log(f"[DEBUG] Copying from bundle resources: {app_resources}")
        safe_log(f"[DEBUG] Copy destination: {temp_copy_dir}")

        if os.path.exists(temp_copy_dir):
            shutil.rmtree(temp_copy_dir, ignore_errors=True)

        shutil.copytree(app_resources, temp_copy_dir, dirs_exist_ok=True)

        if not any(pathlib.Path(temp_copy_dir).iterdir()):
            safe_log("[ERROR] Temp STM32 copy appears empty — copy failed!")

        active_src = temp_copy_dir
    else:
        active_src = PROJECT_SRC


def build_and_flash(
    root,
    show_log_var,
    OPENOCD_INTERFACE,
    OPENOCD_TARGET,
    set_initial_time,
    build_btn=None,
):
    global active_src

    # Create log window if enabled
    log_text = None
    if show_log_var.get():
        log_win = tk.Toplevel(root)
        log_win.title("Build Log")
        log_text = st.ScrolledText(log_win, width=100, height=30)
        log_text.pack(padx=10, pady=10)
        log_text.insert(tk.END, "Starting build...\n")

    def safe_log(msg):
        print(msg.strip())
        if log_text:
            log_text.insert(tk.END, msg + "\n")
            log_text.see(tk.END)
            log_text.update_idletasks()

    safe_log("[DEBUG] Cleaning build directories to force recompilation...")
    cleanup_old_builds(safe_log)

    safe_log("[DEBUG] Ensuring writable copy of source...")
    ensure_writable_copy(safe_log)

    if os.path.exists(BUILD_DIR):
        shutil.rmtree(BUILD_DIR, ignore_errors=True)
    os.makedirs(BUILD_DIR, exist_ok=True)

    internal_build_dir = os.path.join(PROJECT_SRC, "build")
    if os.path.exists(internal_build_dir):
        shutil.rmtree(internal_build_dir, ignore_errors=True)
        safe_log(f"[DEBUG] Removed stale internal build: {internal_build_dir}")
    else:
        safe_log(f"[DEBUG] No internal build directory to remove.")

    try:
        # Debug info
        safe_log(f"[DEBUG] PROJECT_SRC = {PROJECT_SRC}")
        safe_log(f"[DEBUG] TOOLS_DIR   = {TOOLS_DIR}")
        safe_log(f"[DEBUG] BUILD_DIR   = {BUILD_DIR}")
        safe_log(f"[DEBUG] TOOLCHAIN   = {TOOLCHAIN}")
        safe_log(f"[DEBUG] CMAKE       = {CMAKE}")
        safe_log(f"[DEBUG] NINJA       = {NINJA}")
        safe_log(f"[DEBUG] OPENOCD     = {OPENOCD}")
        safe_log(f"[DEBUG] TOOLCHAIN_FILE = {TOOLCHAIN_FILE}")
        safe_log(f"[DEBUG] Using user_config.h at: {ensure_user_config(True)}\n")

        # Remove any stale CMake artifacts from the copied source
        for stale in ["CMakeCache.txt", "CMakeFiles", "build"]:
            stale_path = os.path.join(active_src, stale)
            if os.path.exists(stale_path):
                (
                    shutil.rmtree(stale_path, ignore_errors=True)
                    if os.path.isdir(stale_path)
                    else os.remove(stale_path)
                )
                safe_log(f"[DEBUG] Removed stale CMake file/folder: {stale_path}")
            else:
                safe_log(f"[DEBUG] No stale CMake file/folder to remove: {stale_path}")

        # # Delete any old CMake cache
        # cache_path = os.path.join(BUILD_DIR, "CMakeCache.txt")
        # if os.path.exists(cache_path):
        #     os.remove(cache_path)
        #     safe_log("[DEBUG] Removed CMakeCache.txt for a clean configure.")

        if not change_user_config(root, set_initial_time, safe_log):
            safe_log("[DEBUG] change_user_config() returned False — aborting build.")
            build_btn.config(state="normal")
            return
        safe_log("[DEBUG] change_user_config() completed successfully.")

        # Preview user_config.h
        safe_log("[DEBUG] --- user_config.h preview ---")
        try:
            with open(ensure_user_config(True, safe_log), "r") as f:
                for i, line in enumerate(f.readlines()):
                    if i >= 20:
                        break
                    safe_log(line.strip())
        except Exception as e:
            safe_log(f"[WARN] Could not preview user_config.h: {e}")
            return False
        safe_log("[DEBUG] --- end preview ---\n")

        # --- Run CMake configure ---
        safe_log("[STEP] Running CMake configure...")

        # ensure PATH includes our bundled ninja
        os.environ["PATH"] = f"{os.path.dirname(NINJA)}:" + os.environ.get("PATH", "")

        if (
            run_with_log(
                [
                    CMAKE,
                    "-S",
                    active_src,
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

        # --- Run build ---
        safe_log("\n[STEP] Building firmware...")
        if run_with_log([CMAKE, "--build", BUILD_DIR], log_text) != 0:
            raise subprocess.CalledProcessError(1, "cmake build")

        # --- Find ELF ---
        elf_file = find_elf()
        if not elf_file:
            messagebox.showerror("Flash", "❌ No ELF file found after build.")
            return False
        safe_log(f"\n[DEBUG] Built ELF: {elf_file}")

        # --- Flash device ---
        safe_log("\n[STEP] Flashing device...")
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
        safe_log(f"[DEBUG] Flash command:\n{' '.join(flash_cmd)}\n")

        if run_with_log(flash_cmd, log_text) != 0:
            raise subprocess.CalledProcessError(1, "openocd")

        safe_log("\n[SUCCESS] ✅ Build and flash complete!")

    except subprocess.CalledProcessError as e:
        safe_log(
            f"\n[ERROR] ❌ Command failed: {e}\n[DEBUG] Exception details: {e.args}"
        )
        safe_log("[ERROR] Build/flash process terminated.")
        messagebox.showerror("Error", f"❌ Failed during build/flash: {e}")
        return False

    return True


def dual_build_flash(root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET):
    global set_initial_time
    build_btn.config(state="disabled")
    root.update_idletasks()

    set_initial_time = 1

    if not build_and_flash(
        root,
        show_log_var,
        OPENOCD_INTERFACE,
        OPENOCD_TARGET,
        set_initial_time,
        build_btn,
    ):
        build_btn.config(state="normal")
        return False

    root.update_idletasks()

    set_initial_time = 0

    if not build_and_flash(
        root,
        show_log_var,
        OPENOCD_INTERFACE,
        OPENOCD_TARGET,
        set_initial_time,
        build_btn,
    ):
        build_btn.config(state="normal")
        return False

    root.update_idletasks()

    build_btn.config(state="normal")
    
    return True
