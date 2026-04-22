"""Run decode_fsk on every WAV file in the AUTOTEST folder.

File names must follow the pattern  <f0>-<f1>.WAV  (case-insensitive),
e.g. 10434-11034.WAV  →  f0=10434, f1=11034.
"""

import os
import re
import sys
import time

from demodulator import decode_fsk

AUTOTEST_DIR = os.path.join(os.path.dirname(__file__), "AUTOTEST")
FILENAME_RE = re.compile(r"^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)\.wav$", re.IGNORECASE)


def main():
    wav_files = sorted(
        f for f in os.listdir(AUTOTEST_DIR) if FILENAME_RE.match(f)
    )

    if not wav_files:
        print(f"No matching WAV files found in {AUTOTEST_DIR}")
        sys.exit(1)

    passed = 0
    failed = 0
    t_start = time.monotonic()

    for filename in wav_files:
        m = FILENAME_RE.match(filename)
        f0 = float(m.group(1))
        f1 = float(m.group(2))
        filepath = os.path.join(AUTOTEST_DIR, filename)

        print(f"\n{'='*60}")
        print(f"File   : {filename}")
        print(f"f0={f0:.2f} Hz  f1={f1:.2f} Hz")
        print(f"{'='*60}")

        try:
            count = decode_fsk(
                filepath,
                f0=f0,
                f1=f1,
                generate_debug=False,
                minutes_per_segment=-1,
                use_ecc=True,
                ecc_nsym=20,
            )
            print(f"Result : {count} message(s) decoded")
            passed += 1
        except Exception as exc:
            print(f"ERROR  : {exc}")
            failed += 1

    elapsed = time.monotonic() - t_start
    print(f"\n{'='*60}")
    print(f"Done — {passed} file(s) OK, {failed} file(s) failed")
    print(f"Total time: {elapsed:.1f}s\n")


if __name__ == "__main__":
    main()
