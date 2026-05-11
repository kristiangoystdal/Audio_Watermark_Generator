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

    results = []
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

        t_file = time.monotonic()
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
            elapsed_file = time.monotonic() - t_file
            print(f"Result : {count} message(s) decoded")
            if count > 0:
                results.append((filename, "PASS", count, elapsed_file, ""))
            else:
                print("FAIL   : no messages decoded")
                results.append((filename, "FAIL", 0, elapsed_file, "No messages decoded"))
        except Exception as exc:
            elapsed_file = time.monotonic() - t_file
            print(f"ERROR  : {exc}")
            results.append((filename, "FAIL", 0, elapsed_file, str(exc)))

    elapsed = time.monotonic() - t_start
    passed = sum(1 for r in results if r[1] == "PASS")
    failed = len(results) - passed

    col_file = max(len("File"), max(len(r[0]) for r in results))
    col_status = len("Status")
    col_msgs = len("Messages")
    col_time = len("Time (s)")
    col_note = max(len("Note"), max(len(r[4]) for r in results))

    sep = f"+{'-'*(col_file+2)}+{'-'*(col_status+2)}+{'-'*(col_msgs+2)}+{'-'*(col_time+2)}+{'-'*(col_note+2)}+"
    header = f"| {'File':<{col_file}} | {'Status':<{col_status}} | {'Messages':>{col_msgs}} | {'Time (s)':>{col_time}} | {'Note':<{col_note}} |"

    print(f"\n{sep}")
    print(header)
    print(sep)
    for filename, status, count, t, note in results:
        print(f"| {filename:<{col_file}} | {status:<{col_status}} | {count:>{col_msgs}} | {t:>{col_time}.1f} | {note:<{col_note}} |")
    print(sep)

    if failed == 0:
        print(f"\nALL {passed} TESTS PASSED  ({elapsed:.1f}s)\n")
    else:
        failed_files = ", ".join(r[0] for r in results if r[1] == "FAIL")
        print(f"\n{passed} PASSED, {failed} FAILED: {failed_files}  ({elapsed:.1f}s)\n")


if __name__ == "__main__":
    main()
