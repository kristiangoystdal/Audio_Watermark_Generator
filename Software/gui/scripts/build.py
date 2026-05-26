import os
import shutil
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox
import tkinter.scrolledtext as st
import tempfile
import pathlib
import serial
import serial.tools.list_ports
from datetime import datetime, UTC

from scripts.paths import *
from scripts.user_config import *

active_src = PROJECT_SRC
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


def flash_only(log_text, safe_log, leave=True):
    elf_file = find_elf()
    if not elf_file:
        messagebox.showerror("Flash", "❌ No ELF file found after build.")
        return False

    safe_log(f"\n[DEBUG] Built ELF: {elf_file}")

    # Convert ELF to BIN
    bin_file = elf_file.replace(".elf", ".bin")
    safe_log("\n[STEP] Converting ELF to BIN...")

    objcopy = os.path.join(TOOLCHAIN, "bin", "arm-none-eabi-objcopy")
    if not os.path.exists(objcopy):
        objcopy = "arm-none-eabi-objcopy"

    if run_with_log([objcopy, "-O", "binary", elf_file, bin_file], log_text) != 0:
        raise subprocess.CalledProcessError(1, "objcopy")

    # Use bundled dfu-util, fall back to system
    dfu_util = DFU_UTIL if os.path.exists(DFU_UTIL) else "dfu-util"
    safe_log(f"\n[STEP] Flashing via USB DFU using {dfu_util}...")
    safe_log("[INFO] Make sure BOOT0 is HIGH and device is in DFU mode")

    start_addr = "0x08000000:leave" if leave else "0x08000000"
    if (
        run_with_log(
            [dfu_util, "-a", "0", "-s", start_addr, "-D", bin_file],
            log_text,
            success_if_output_contains="File downloaded successfully",
        )
        != 0
    ):
        raise subprocess.CalledProcessError(1, "dfu-util")

    safe_log("\n[SUCCESS] Flash complete!")
    return True


def run_with_log(cmd, log_text=None, success_if_output_contains=None):
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        output = []
        for line in process.stdout:
            output.append(line)
            if log_text:
                log_text.insert(tk.END, line)
                log_text.see(tk.END)
                log_text.update_idletasks()
            print(line, end="")

        process.wait()
        combined_output = "".join(output)

        if process.returncode != 0:
            if (
                success_if_output_contains
                and success_if_output_contains in combined_output
            ):
                return 0
            err_msg = (
                f"\n[ERROR] Command failed ({cmd[0]} exited with {process.returncode})\n"
                + "".join(output[-50:])
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


def clear_flash_flag(log_text, safe_log):
    safe_log("\n[STEP] Clearing RTC flash flag page...")

    # Create a 2KB blank binary (one flash page of 0xFF)
    blank_page = bytes([0xFF] * 2048)

    tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    tmp.write(blank_page)
    tmp.close()

    dfu_util = DFU_UTIL if os.path.exists(DFU_UTIL) else "dfu-util"

    result = run_with_log(
        [dfu_util, "-a", "0", "-s", "0x0801F800", "-D", tmp.name],
        log_text,
        success_if_output_contains="File downloaded successfully",
    )

    os.unlink(tmp.name)

    if result != 0:
        safe_log("[WARN] Could not clear flash flag — RTC time may not be reset.")
    else:
        safe_log("[OK] Flash flag cleared.")


def send_time_sync(root, safe_log):
    safe_log("\n[STEP] Waiting for device to boot into firmware...")

    messagebox.showinfo(
        "Toggle BOOT0",
        "Flash complete!\n\nSet BOOT0 LOW and power-cycle. Click OK when done.",
    )

    port = None
    for attempt in range(60):
        all_ports = serial.tools.list_ports.comports()
        for p in all_ports:
            if any(
                x in (p.description or p.hwid or p.device or "").lower()
                for x in ["stm32", "usbmodem", "0483:5740"]
            ):
                port = p.device
                break
        if port:
            break
        time.sleep(0.1)

    if not port:
        safe_log("[WARN] STM32 CDC port not found, skipping time sync")
        return False

    try:
        with serial.Serial(port, 115200, timeout=10) as ser:
            time.sleep(2)
            safe_log(f"[INFO] Listening on {port}...")

            ready = False
            start = time.time()
            while time.time() - start < 10:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        safe_log(f"[DEBUG] {line}")
                    if "READY_FOR_TIME_SYNC" in line:
                        ready = True
                        break
                except:
                    pass

            if not ready:
                safe_log("[WARN] Never received READY_FOR_TIME_SYNC")
                return False

            # Sample time and send
            sent_time = datetime.now(UTC)
            dow = (sent_time.weekday() + 1) % 7
            time_str = f"T{sent_time.hour:02d}:{sent_time.minute:02d}:{sent_time.second:02d} {dow} {sent_time.day:02d}/{sent_time.month:02d}/{sent_time.year}\n"

            safe_log(
                f"[INFO] Sent time: {sent_time.hour:02d}:{sent_time.minute:02d}:{sent_time.second:02d}"
            )
            ser.write(time_str.encode())

            # Wait for confirmation
            start = time.time()
            while time.time() - start < 10:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        safe_log(f"[DEBUG] {line}")
                    if "Time synced:" in line:
                        safe_log("[INFO] Time sync successful!")
                        return True
                except:
                    pass

            return False

    except Exception as e:
        safe_log(f"[WARN] Time sync failed: {e}")
        return False


def build_flash(root, show_log_var, build_btn):
    build_btn.config(state="disabled")
    root.update_idletasks()

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

    try:
        prepare_build(log_text, safe_log)

        safe_log("\n========== Build and Flash ==========")
        if not change_user_config(
            root, 0, safe_log
        ):  # SET_INITIAL_TIME=0, handled via UART now
            raise Exception("Failed to update user_config")
        build_only(log_text, safe_log)
        clear_flash_flag(log_text, safe_log)
        flash_only(log_text, safe_log, leave=True)  # don't leave, we detach manually
        time_sync_ok = send_time_sync(root, safe_log)

        safe_log("\n[SUCCESS] ✅ Build & flash complete!")

        if not time_sync_ok:
            build_btn.config(state="normal")
            return 1

    except Exception as e:
        safe_log(f"\n[ERROR] ❌ {e}")
        messagebox.showerror("Error", f"Build/flash failed: {e}")
        build_btn.config(state="normal")
        return 2

    build_btn.config(state="normal")
    return 0
