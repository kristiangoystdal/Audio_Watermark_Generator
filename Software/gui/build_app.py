#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path

SPEC_FILE = "STM32Tool.spec"


def clean():
    for folder in ["build", "dist"]:
        path = Path(folder)
        if path.exists():
            print(f"Removing {folder}/")
            subprocess.run(
                ["xattr", "-cr", str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            subprocess.run(["rm", "-rf", str(path)], check=True)


def build():
    print("\n>>> Running PyInstaller\n")
    result = subprocess.run(["pyinstaller", SPEC_FILE, "--noconfirm"])
    if result.returncode != 0:
        print("\n❌ Build failed.")
        sys.exit(result.returncode)
    print("\n✅ Build completed successfully.")


if __name__ == "__main__":
    clean()
    build()
