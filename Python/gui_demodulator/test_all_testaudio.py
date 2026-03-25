#!/usr/bin/env python3

import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TESTAUDIO_DIR = ROOT / "TESTAUDIO"
DEMODULATOR = ROOT / "demodulator.py"
FILENAME_PATTERN = re.compile(r"^(?P<f0>\d+)-(?P<f1>\d+)\.wav$", re.IGNORECASE)


def iter_wav_jobs():
    for wav_path in sorted(TESTAUDIO_DIR.glob("*.WAV")) + sorted(TESTAUDIO_DIR.glob("*.wav")):
        match = FILENAME_PATTERN.match(wav_path.name)
        if not match:
            print(f"Skipping {wav_path.name}: filename does not match <f0>-<f1>.WAV")
            continue
        yield wav_path, int(match.group("f0")), int(match.group("f1"))


def main() -> int:
    if not TESTAUDIO_DIR.is_dir():
        print(f"Missing directory: {TESTAUDIO_DIR}", file=sys.stderr)
        return 1
    if not DEMODULATOR.is_file():
        print(f"Missing file: {DEMODULATOR}", file=sys.stderr)
        return 1

    jobs = list(iter_wav_jobs())
    if not jobs:
        print("No matching WAV files found in TESTAUDIO.")
        return 1

    failures = 0
    for index, (wav_path, f0, f1) in enumerate(jobs, start=1):
        print(f"\n[{index}/{len(jobs)}] Processing {wav_path.name} with f0={f0}, f1={f1}")
        cmd = [
            sys.executable,
            str(DEMODULATOR),
            "-i",
            str(wav_path),
            "--f0",
            str(f0),
            "--f1",
            str(f1),
            "--use-ecc",
            "--ecc-parity-bytes",
            "20",
            "--generate-debug",
            "--minutes-per-segment",
            "-1",
        ]
        result = subprocess.run(cmd, cwd=ROOT)
        if result.returncode != 0:
            failures += 1
            print(f"Failed: {wav_path.name} exited with code {result.returncode}")

    print(f"\nCompleted {len(jobs)} file(s). Failures: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
