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
active_build_dir = BUILD_DIR

os.environ["PATH"] = (
    f"{os.path.abspath(os.path.join(os.path.dirname(sys.executable), '..', 'Resources', 'tools', 'ninja'))}:"
    + os.environ.get("PATH", "")
)


def prepare_build(log_text, safe_log):
    global active_src

    safe_log("[DEBUG] Ensuring writable copy of source...")
    ensure_writable_copy(safe_log)

    os.makedirs(active_build_dir, exist_ok=True)

    safe_log("[STEP] Running CMake configure (only if needed)...")

    os.environ["PATH"] = f"{os.path.dirname(NINJA)}:" + os.environ.get("PATH", "")

    if (
        run_with_log(
            [
                CMAKE,
                "-S",
                active_src,
                "-B",
                active_build_dir,
                "-G",
                "Ninja",
                f"-DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN_FILE}",
                f"-DCMAKE_MAKE_PROGRAM={NINJA}",
                "-DCMAKE_BUILD_TYPE=MinSizeRel",
            ],
            log_text,
        )
        != 0
    ):
        raise subprocess.CalledProcessError(1, "cmake configure")


def build_only(log_text, safe_log):
    safe_log("\n[STEP] Building firmware (incremental)...")

    if (
        run_with_log(
            [CMAKE, "--build", active_build_dir, "--parallel"],
            log_text,
        )
        != 0
    ):
        raise subprocess.CalledProcessError(1, "cmake build")


def flash_only(log_text, safe_log, OPENOCD_INTERFACE, OPENOCD_TARGET):
    elf_file = find_elf()
    if not elf_file:
        messagebox.showerror("Flash", "❌ No ELF file found after build.")
        return False

    safe_log(f"\n[DEBUG] Built ELF: {elf_file}")
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
        "adapter speed 4000",
        "-c",
        "reset_config srst_only srst_nogate connect_assert_srst",
        "-c",
        f"program {elf_file} verify reset exit",
    ]

    if run_with_log(flash_cmd, log_text) != 0:
        raise subprocess.CalledProcessError(1, "openocd")

    safe_log("\n[SUCCESS] Flash complete!")
    return True


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
        active_build_dir,
        os.path.join(active_build_dir, "build"),
        os.path.join(active_build_dir, "build", "Debug"),
        os.path.join(active_build_dir, "build", "Release"),
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


def ensure_writable_copy(safe_log):
    global active_src, active_build_dir

    if getattr(sys, "frozen", False) and sys.platform == "darwin":
        temp_root = tempfile.gettempdir()
        temp_copy_dir = os.path.join(temp_root, "stm32_src")
        temp_build_dir = os.path.join(temp_root, "stm32_build")

        app_resources = os.path.abspath(
            os.path.join(os.path.dirname(sys.executable), "..", "Resources", "STM32")
        )

        safe_log(f"[DEBUG] Copying from bundle resources: {app_resources}")
        safe_log(f"[DEBUG] Copy destination: {temp_copy_dir}")
        safe_log(f"[DEBUG] Build directory: {temp_build_dir}")

        if os.path.exists(temp_copy_dir):
            shutil.rmtree(temp_copy_dir, ignore_errors=True)
        if os.path.exists(temp_build_dir):
            shutil.rmtree(temp_build_dir, ignore_errors=True)

        shutil.copytree(app_resources, temp_copy_dir, dirs_exist_ok=True)

        if not any(pathlib.Path(temp_copy_dir).iterdir()):
            safe_log("[ERROR] Temp STM32 copy appears empty — copy failed!")

        active_src = temp_copy_dir
        active_build_dir = temp_build_dir
    else:
        active_src = PROJECT_SRC
        active_build_dir = BUILD_DIR


def dual_build_flash(root, show_log_var, build_btn, OPENOCD_INTERFACE, OPENOCD_TARGET):
    global set_initial_time

    build_btn.config(state="disabled")
    root.update_idletasks()

    log_text = None
    if show_log_var.get():
        log_win = tk.Toplevel(root)
        log_win.title("Build Log")
        log_text = st.ScrolledText(log_win, width=100, height=30)
        log_text.pack(padx=10, pady=10)
        log_text.insert(tk.END, "Starting dual build...\n")

    def safe_log(msg):
        print(msg.strip())
        if log_text:
            log_text.insert(tk.END, msg + "\n")
            log_text.see(tk.END)
            log_text.update_idletasks()

    try:
        # Configure once
        prepare_build(log_text, safe_log)

        # ---------------------------
        # FIRST BUILD (set RTC time)
        # ---------------------------
        safe_log("\n========== PASS 1: SET INITIAL TIME ==========")
        set_initial_time = 1

        if not change_user_config(root, set_initial_time, safe_log):
            raise Exception("Failed to update user_config")

        build_only(log_text, safe_log)
        flash_only(log_text, safe_log, OPENOCD_INTERFACE, OPENOCD_TARGET)

        # ---------------------------
        # SECOND BUILD (normal mode)
        # ---------------------------
        safe_log("\n========== PASS 2: NORMAL MODE ==========")
        set_initial_time = 0

        if not change_user_config(root, set_initial_time, safe_log):
            raise Exception("Failed to update user_config")

        build_only(log_text, safe_log)
        flash_only(log_text, safe_log, OPENOCD_INTERFACE, OPENOCD_TARGET)

        safe_log("\n[SUCCESS] ✅ Dual build & flash complete!")

    except Exception as e:
        safe_log(f"\n[ERROR] ❌ {e}")
        messagebox.showerror("Error", f"Build/flash failed: {e}")
        build_btn.config(state="normal")
        return False

    build_btn.config(state="normal")
    return True
