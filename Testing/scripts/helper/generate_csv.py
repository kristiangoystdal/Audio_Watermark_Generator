#!/usr/bin/env python3
"""
Generate coords.csv and detections.csv from audio files in subdirectories.
Extracts coordinates from filenames like: mic1_x0_y0.wav, mic2_x3.74_y3.74.wav
"""

import os
import re
from pathlib import Path
import pandas as pd

_XY_RE = re.compile(r"x(-?\d+(?:\.\d+)?)_y(-?\d+(?:\.\d+)?)", re.IGNORECASE)


def parse_position(filename):
    """Extract (x, y) from filename like mic_x3.74_y2.43.wav."""
    m = _XY_RE.search(os.path.basename(filename))
    if m:
        return float(m.group(1)), float(m.group(2))
    return None, None


def generate_csvs_for_subfolder(subfolder_path, class_name="human"):
    """Generate coords.csv and detections.csv for a subfolder."""

    # Find all wav files
    wav_files = []
    for f in sorted(Path(subfolder_path).glob("*.wav")) + sorted(
        Path(subfolder_path).glob("*.WAV")
    ):
        wav_files.append(str(f))

    if not wav_files:
        print(f"  ⚠️  No .wav files found")
        return False

    # Extract positions
    coords_data = []
    detections_data = []

    for path in wav_files:
        filename = os.path.basename(path)
        x, y = parse_position(path)

        if x is None or y is None:
            print(f"  ⚠️  Could not parse x/y from {filename}, skipping")
            continue

        coords_data.append({"filename": filename, "x": x, "y": y})

        # Create detection record (entire file as one detection)
        # Note: using 'file' and 'class' for OpenSoundscape compatibility
        detections_data.append(
            {
                "detector": "auto",
                "start_time": 0.0,
                "end_time": 999.0,  # Entire file
                "start_timestamp": "2024-01-01 00:00:00+00:00",  # ISO format with timezone
                "file": filename,
                "class": class_name,
            }
        )

    if not coords_data:
        print(f"  ⚠️  No valid coordinates found")
        return False

    # Save coords.csv
    coords_df = pd.DataFrame(coords_data)
    coords_path = os.path.join(subfolder_path, "coords.csv")
    coords_df.to_csv(coords_path, index=False)
    print(f"  ✓ Created {coords_path}")
    print(f"    {len(coords_data)} microphone(s)")

    # Save detections.csv
    detections_df = pd.DataFrame(detections_data)
    detections_path = os.path.join(subfolder_path, "detections.csv")
    detections_df.to_csv(detections_path, index=False)
    print(f"  ✓ Created {detections_path}")
    print(f"    {len(detections_data)} detection(s) with class '{class_name}'")

    return True


def main():
    base_folder = os.path.join("test_files", "direction")

    if not os.path.isdir(base_folder):
        print(f"❌ Folder not found: {base_folder}")
        return

    # Find all subfolders
    subfolders = sorted(
        [entry.name for entry in os.scandir(base_folder) if entry.is_dir()]
    )

    if not subfolders:
        print(f"❌ No subfolders found in {base_folder}")
        return

    print(f"📂 Found {len(subfolders)} subfolder(s)\n")

    # Ask for class name
    class_name = input("Enter class name for detections (default: 'human'): ").strip()
    if not class_name:
        class_name = "human"

    print(f"\n🔧 Generating CSVs with class '{class_name}'...\n")

    success_count = 0
    for subfolder_name in subfolders:
        subfolder_path = os.path.join(base_folder, subfolder_name)
        print(f"📂 {subfolder_name}:")

        if generate_csvs_for_subfolder(subfolder_path, class_name):
            success_count += 1
        print()

    print(f"✅ Done! Generated CSVs for {success_count}/{len(subfolders)} subfolder(s)")


if __name__ == "__main__":
    main()
