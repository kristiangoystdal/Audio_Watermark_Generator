import os
import sys
import tempfile
import tkinter.messagebox as messagebox

# ---------------------------------------------------------
# Paths
# ---------------------------------------------------------

if getattr(sys, "frozen", False):
    # MacOS .app build
    if sys.platform == "darwin":
        RESOURCES_DIR = os.path.abspath(
            os.path.join(os.path.dirname(sys.executable), "..", "Resources")
        )
        PROJECT_SRC = os.path.join(RESOURCES_DIR, "STM32")
        TOOLS_DIR = os.path.join(RESOURCES_DIR, "tools")

        if not os.path.exists(PROJECT_SRC):
            messagebox.showerror("Error", f"STM32 folder not found: {PROJECT_SRC}")
            sys.exit(1)

    # Windows/Linux build
    else:
        BASE = sys._MEIPASS
        PROJECT_SRC = os.path.join(BASE, "STM32")
        TOOLS_DIR = os.path.join(BASE, "tools")
else:
    # Running from source
    BASE = os.path.dirname(__file__)
    PROJECT_SRC = os.path.abspath(os.path.join(BASE, "../../../STM32"))
    TOOLS_DIR = os.path.join(BASE, "..", "tools")


BUILD_DIR = os.path.join(tempfile.gettempdir(), "stm32_build")

TOOLCHAIN = os.path.join(TOOLS_DIR, "arm-none-eabi-gcc")

cmake_root = os.path.join(TOOLS_DIR, "cmake")
if os.path.exists(cmake_root):
    cmake_versions = [
        d
        for d in os.listdir(cmake_root)
        if os.path.isdir(os.path.join(cmake_root, d)) and d != "arm-gcc-toolchain.cmake"
    ]
    cmake_version_dir = cmake_versions[0] if cmake_versions else "4.1.2"
else:
    cmake_version_dir = "4.1.2"

CMAKE = os.path.join(cmake_root, cmake_version_dir, "bin", "cmake")
TOOLCHAIN_FILE = os.path.join(TOOLS_DIR, "cmake", "arm-gcc-toolchain.cmake")
NINJA = os.path.join(TOOLS_DIR, "ninja", "ninja")
DFU_UTIL = os.path.join(TOOLS_DIR, "dfu-util", "dfu-util")
