import os
import numpy as np
import matplotlib.pyplot as plt


test_files_folder_path = "test_files"
results_folder_path = "results"
os.makedirs(results_folder_path, exist_ok=True)

# Find all .wav files in "audio_files"/folder and subfolders and save the paths to a list
def find_wav_files(folder):
    """Recursively find all .wav files in the specified folder in the "test_files" directory and return their paths."""
    test_files_folder = os.path.join(test_files_folder_path, folder)
    print(f"Searching for .wav files in: {test_files_folder}")
    wav_file_paths = []
    for root, dirs, files in os.walk(test_files_folder):
        for file in files:
            if file.lower().endswith(".wav"):
                wav_file_paths.append(os.path.join(root, file))
    return wav_file_paths


# Find all .txt files in "timing"/folder and subfolders and save the paths to a list
def find_txt_files(folder):
    test_files_folder = os.path.join(test_files_folder_path, folder)
    print(f"Searching for .txt files in: {test_files_folder}")
    txt_file_paths = []
    for root, dirs, files in os.walk(test_files_folder):
        for file in files:
            if file.lower().endswith(".txt"):
                txt_file_paths.append(os.path.join(root, file))
    return txt_file_paths


def extract_subfolder_name(file_path):
    return file_path.split("/")[-2]


def save_plot_to_results_folder(plot, subfolder, filename):
    """Save a plot to the results folder."""
    results_folder = os.path.join(results_folder_path, subfolder)
    os.makedirs(results_folder, exist_ok=True)
    plot_path = os.path.join(results_folder, filename)
    plot.savefig(plot_path)
    print(f"Plot saved to: {plot_path}")


def copy_txt_to_decoded_results(subfolder):
    """Copy all .txt files from 01_test_files to 02_results/decoded/, preserving subfolder structure."""
    import shutil

    src_root = os.path.join(test_files_folder_path, subfolder)
    dst_root = os.path.join(results_folder_path, "decoded", subfolder)
    copied = 0
    for root, dirs, files in os.walk(src_root):
        dirs.sort()
        for file in sorted(files):
            if file.lower().endswith(".txt"):
                src_path = os.path.join(root, file)
                rel = os.path.relpath(src_path, src_root)
                dst_path = os.path.join(dst_root, rel)
                os.makedirs(os.path.dirname(dst_path), exist_ok=True)
                shutil.copy2(src_path, dst_path)
                print(f"Copied: {src_path}  ->  {dst_path}")
                copied += 1
    print(f"\n{copied} file(s) copied to {dst_root}")


def save_results_to_csv(headers, data, subfolder, filename):
    """Save results to a CSV file with headers in the results folder."""
    import csv

    results_folder = os.path.join(results_folder_path, subfolder)
    os.makedirs(results_folder, exist_ok=True)
    csv_path = os.path.join(results_folder, filename)

    with open(csv_path, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        writer.writerows(row.values() for row in data)

    print(f"Results saved to: {csv_path}")
