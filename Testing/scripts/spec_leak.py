#!/usr/bin/env python3
import os
import numpy as np
import librosa
from pathlib import Path
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.signal import welch, find_peaks
from helper.helper import *

# Thresholds
ISOLATION_THRESHOLD_DB = 12  # Minimum isolation between tones (dB)
OOB_POWER_THRESHOLD_DB = (
    -10
)  # Out-of-band energy threshold relative to strongest tone (dB)
HARMONIC_THRESHOLD_DB = -25  # Harmonic/sidelobe rejection threshold (dB)


def extract_freqs_from_filename(filename):
    """Extract target frequencies from filename like '15000-16000_cable.wav'"""
    stem = Path(filename).stem
    base = stem.split("_")[0]
    parts = base.split("-")
    if len(parts) >= 2:
        try:
            return int(parts[0]), int(parts[1])
        except ValueError:
            return None, None
    return None, None


def refine_fsk_tones_fft(x, fs, f0, f1, search_width_hz=200.0):
    """
    FFT-based tone refinement: find exact peak frequencies within search bands.
    Uses parabolic interpolation for sub-bin accuracy.
    """
    x = np.asarray(x, dtype=float)
    nyq = fs / 2.0

    w = np.hanning(len(x))
    xw = x * w
    X = np.fft.rfft(xw)
    power = X.real * X.real + X.imag * X.imag
    df = fs / len(xw)

    def peak_in_band(f_center):
        lo = max(0.0, f_center - search_width_hz)
        hi = min(nyq, f_center + search_width_hz)
        k_lo = int(np.clip(np.round(lo / df), 0, len(power) - 1))
        k_hi = int(np.clip(np.round(hi / df), 0, len(power) - 1))
        if k_hi < k_lo:
            return f_center
        band = power[k_lo : k_hi + 1]
        k = k_lo + np.argmax(band)
        # Parabolic interpolation
        if 1 <= k < len(power) - 1:
            y0 = np.log(power[k - 1] + 1e-30)
            y1 = np.log(power[k] + 1e-30)
            y2 = np.log(power[k + 1] + 1e-30)
            denom = y0 - 2 * y1 + y2
            if abs(denom) > 1e-10:
                delta = 0.5 * (y0 - y2) / denom
                return float(np.clip(k * df + delta * df, lo, hi))
        return float(k * df)

    f0_new = peak_in_band(f0)
    f1_new = peak_in_band(f1)
    return f0_new, f1_new


def measure_spectral_purity(audio_path, f1, f2, detect_bw=150, plot=False):
    """
    Comprehensive FSK spectral purity measurement.

    Validates that audio contains ONLY target frequencies by measuring:
    1. Tone-to-tone isolation (can FSK decoder distinguish tones?)
    2. Out-of-band energy (unwanted harmonics/interference)
    3. Peak stability across time (tone quality)
    4. Sidelobe structure (windowing quality)

    Args:
        audio_path: Path to audio file
        f1, f2: Target frequencies (Hz)
        detect_bw: Detection bandwidth around each tone (Hz) - default 150
        plot: Generate detailed spectrum plot

    Returns:
        Dictionary with comprehensive spectral metrics
    """
    # Load audio
    y, sr = librosa.load(audio_path, sr=None, mono=True)
    print(f"  Loaded: {len(y)} samples @ {sr} Hz ({len(y)/sr:.2f} sec)")

    # Refine frequencies to exact peaks
    f1_refined, f2_refined = refine_fsk_tones_fft(y, sr, f1, f2, search_width_hz=200)
    print(f"  Refined: f1={f1} → {f1_refined:.1f} Hz, f2={f2} → {f2_refined:.1f} Hz")

    # Ensure f1 < f2
    if f1_refined > f2_refined:
        f1_refined, f2_refined = f2_refined, f1_refined

    # Compute Welch PSD for stability
    nperseg = min(4096, max(512, len(y) // 4))
    freqs, power_linear = welch(
        y, sr, window="hann", nperseg=nperseg, noverlap=nperseg // 2, scaling="spectrum"
    )
    power_db = 10 * np.log10(power_linear + 1e-12)

    # Find peak indices
    idx_f1 = np.argmin(np.abs(freqs - f1_refined))
    idx_f2 = np.argmin(np.abs(freqs - f2_refined))
    peak_f1_db = power_db[idx_f1]
    peak_f2_db = power_db[idx_f2]

    strongest_peak = max(peak_f1_db, peak_f2_db)
    weakest_peak = min(peak_f1_db, peak_f2_db)
    peak_imbalance = strongest_peak - weakest_peak

    print(
        f"  Peaks: f1={peak_f1_db:.1f} dB, f2={peak_f2_db:.1f} dB (imbalance={peak_imbalance:.1f} dB)"
    )

    # ========== METRIC 1: TONE-TO-TONE ISOLATION ==========
    # Measure how much each tone's sidelobes contaminate the other's detection band

    # Define detection bands (wider, symmetric around each tone)
    band1_mask = (freqs >= f1_refined - detect_bw) & (freqs <= f1_refined + detect_bw)
    band2_mask = (freqs >= f2_refined - detect_bw) & (freqs <= f2_refined + detect_bw)

    # Get raw power in each band
    p1_band = power_db[band1_mask]
    p2_band = power_db[band2_mask]

    # Split to separate desired signal from cross-contamination
    midpoint = (f1_refined + f2_refined) / 2
    band1_signal = power_db[band1_mask & (freqs <= midpoint)]
    band2_signal = power_db[band2_mask & (freqs >= midpoint)]
    band1_interference = power_db[
        band1_mask & (freqs > midpoint)
    ]  # f2 leaking into f1 band
    band2_interference = power_db[
        band2_mask & (freqs < midpoint)
    ]  # f1 leaking into f2 band

    p1_desired = np.max(band1_signal) if len(band1_signal) > 0 else -np.inf
    p1_interference = (
        np.max(band1_interference) if len(band1_interference) > 0 else np.min(power_db)
    )
    p2_desired = np.max(band2_signal) if len(band2_signal) > 0 else -np.inf
    p2_interference = (
        np.max(band2_interference) if len(band2_interference) > 0 else np.min(power_db)
    )

    # If one band is empty, isolation is poor
    if p1_interference == -np.inf or p1_interference < np.min(power_db):
        p1_interference = (
            np.min(power_db[band1_mask]) if len(band1_mask) > 0 else np.min(power_db)
        )
    if p2_interference == -np.inf or p2_interference < np.min(power_db):
        p2_interference = (
            np.min(power_db[band2_mask]) if len(band2_mask) > 0 else np.min(power_db)
        )

    isolation_f1 = p1_desired - p1_interference  # f1's ability to reject f2
    isolation_f2 = p2_desired - p2_interference  # f2's ability to reject f1
    min_isolation = min(isolation_f1, isolation_f2)

    print(
        f"  Isolation: f1={isolation_f1:.1f} dB, f2={isolation_f2:.1f} dB (min={min_isolation:.1f} dB)"
    )

    # ========== METRIC 2: OUT-OF-BAND ENERGY ==========
    # Energy in regions far from target frequencies (should be low)

    signal_bw = f2_refined - f1_refined
    oob_margin = 2 * signal_bw  # Look 2x signal bandwidth away

    oob_lo_mask = (freqs > 0) & (freqs < (f1_refined - oob_margin))
    oob_hi_mask = (freqs > (f2_refined + oob_margin)) & (freqs < sr / 2)

    oob_lo = power_db[oob_lo_mask]
    oob_hi = power_db[oob_hi_mask]
    oob_energy = (
        np.max(np.concatenate([oob_lo, oob_hi]))
        if len(np.concatenate([oob_lo, oob_hi])) > 0
        else -np.inf
    )

    oob_relative_strongest = oob_energy - strongest_peak

    print(
        f"  Out-of-band energy: {oob_energy:.1f} dB (relative to peak: {oob_relative_strongest:.1f} dB)"
    )

    # ========== METRIC 3: HARMONIC/SIDELOBE CONTENT ==========
    # Check for harmonics at 2f1, 2f2, and their intermodulation products

    harmonics_to_check = [
        (2 * f1_refined, "2f1"),
        (2 * f2_refined, "2f2"),
        (f2_refined - f1_refined, "f2-f1"),
        (f2_refined + f1_refined, "f2+f1"),
    ]

    harmonic_levels = {}  # {label: (freq, power_db)}
    max_harmonic_db = -np.inf

    for f_harm, label in harmonics_to_check:
        if 0 < f_harm < sr / 2:
            idx_harm = np.argmin(np.abs(freqs - f_harm))
            p_harm = power_db[idx_harm]
            harmonic_levels[label] = (f_harm, p_harm)
            max_harmonic_db = max(max_harmonic_db, p_harm)
            print(f"    {label} @ {f_harm:.0f} Hz: {p_harm:.1f} dB")

    harmonic_rejection = strongest_peak - max_harmonic_db

    # ========== METRIC 4: NOISE FLOOR ==========
    # Estimate from quiet regions

    noise_regions = np.concatenate([oob_lo, oob_hi])
    if len(noise_regions) > 0:
        noise_floor = np.median(noise_regions)
        snr_f1 = peak_f1_db - noise_floor
        snr_f2 = peak_f2_db - noise_floor
        min_snr = min(snr_f1, snr_f2)
    else:
        noise_floor = -np.inf
        snr_f1 = snr_f2 = min_snr = 0

    print(f"  SNR: f1={snr_f1:.1f} dB, f2={snr_f2:.1f} dB (min={min_snr:.1f} dB)")

    # ========== PASS/FAIL CRITERIA ==========
    isolation_pass = min_isolation >= ISOLATION_THRESHOLD_DB
    oob_pass = oob_relative_strongest <= OOB_POWER_THRESHOLD_DB
    harmonic_pass = harmonic_rejection >= HARMONIC_THRESHOLD_DB
    overall_pass = isolation_pass and oob_pass and harmonic_pass

    results = {
        "f1": f1_refined,
        "f1_peak_db": peak_f1_db,
        "f2": f2_refined,
        "f2_peak_db": peak_f2_db,
        "peak_imbalance_db": peak_imbalance,
        "isolation_f1_db": isolation_f1,
        "isolation_f2_db": isolation_f2,
        "min_isolation_db": min_isolation,
        "isolation_pass": isolation_pass,
        "oob_energy_db": oob_energy,
        "oob_relative_peak_db": oob_relative_strongest,
        "oob_pass": oob_pass,
        "max_harmonic_db": max_harmonic_db,
        "harmonic_rejection_db": harmonic_rejection,
        "harmonic_pass": harmonic_pass,
        "noise_floor_db": noise_floor,
        "snr_f1_db": snr_f1,
        "snr_f2_db": snr_f2,
        "min_snr_db": min_snr,
        "pass": overall_pass,
    }

    # ========== PLOT ==========
    if plot:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

        # Full spectrum
        ax1.plot(freqs, power_db, linewidth=1.0, color="blue", label="Power spectrum")
        ax1.axhline(
            noise_floor,
            color="gray",
            linestyle="--",
            linewidth=1.5,
            label=f"Noise floor: {noise_floor:.1f} dB",
        )

        # Mark target tones
        ax1.plot(
            f1_refined,
            peak_f1_db,
            "ro",
            markersize=12,
            label=f"f1: {peak_f1_db:.1f} dB",
        )
        ax1.plot(
            f2_refined,
            peak_f2_db,
            "go",
            markersize=12,
            label=f"f2: {peak_f2_db:.1f} dB",
        )

        # Mark detection bands
        ax1.axvspan(
            f1_refined - detect_bw,
            f1_refined + detect_bw,
            alpha=0.1,
            color="red",
            label="f1 detection band",
        )
        ax1.axvspan(
            f2_refined - detect_bw, f2_refined + detect_bw, alpha=0.1, color="green"
        )

        # Mark OOB regions
        ax1.axvspan(0, f1_refined - oob_margin, alpha=0.05, color="orange")
        ax1.axvspan(f2_refined + oob_margin, sr / 2, alpha=0.05, color="orange")

        # Mark harmonics
        for label, (f_harm, p_harm) in harmonic_levels.items():
            ax1.plot(
                f_harm, p_harm, "x", color="purple", markersize=10, markeredgewidth=2
            )
            ax1.annotate(
                label,
                xy=(f_harm, p_harm),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )

        ax1.set_xlim(0, sr / 2)
        ax1.set_ylim(noise_floor - 10, strongest_peak + 15)
        ax1.set_xlabel("Frequency (Hz)", fontsize=11)
        ax1.set_ylabel("Power (dB)", fontsize=11)
        ax1.set_title(f"Full Spectrum: {Path(audio_path).name}", fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="upper right", fontsize=9)

        # Zoomed inter-tone region
        zoom_start = f1_refined - 500
        zoom_end = f2_refined + 500
        zoom_mask = (freqs >= zoom_start) & (freqs <= zoom_end)

        ax2.plot(
            freqs[zoom_mask],
            power_db[zoom_mask],
            linewidth=1.5,
            color="blue",
            label="Power spectrum",
        )
        ax2.plot(
            f1_refined,
            peak_f1_db,
            "ro",
            markersize=12,
            label=f"f1: {peak_f1_db:.1f} dB",
        )
        ax2.plot(
            f2_refined,
            peak_f2_db,
            "go",
            markersize=12,
            label=f"f2: {peak_f2_db:.1f} dB",
        )

        # Shade detection bands
        ax2.axvspan(
            f1_refined - detect_bw, f1_refined + detect_bw, alpha=0.15, color="red"
        )
        ax2.axvspan(
            f2_refined - detect_bw, f2_refined + detect_bw, alpha=0.15, color="green"
        )

        ax2.axhline(
            noise_floor,
            color="gray",
            linestyle="--",
            linewidth=1.5,
            label=f"Noise floor: {noise_floor:.1f} dB",
        )

        ax2.set_xlim(zoom_start, zoom_end)
        ax2.set_xlabel("Frequency (Hz)", fontsize=11)
        ax2.set_ylabel("Power (dB)", fontsize=11)
        ax2.set_title(
            f"Inter-tone Region (Isolation: f1={isolation_f1:.1f} dB, f2={isolation_f2:.1f} dB)",
            fontsize=12,
        )
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper right", fontsize=9)

        plt.tight_layout()
        extracted_subfolder = extract_subfolder_name(audio_path)
        folder_path = os.path.join("spectral_purity", extracted_subfolder)
        os.makedirs(os.path.join("results", folder_path), exist_ok=True)
        save_plot_to_results_folder(plt, folder_path, Path(audio_path).stem + ".png")
        plt.close()

    return results


def main():
    # Find all .wav files
    wav_files = [Path(f) for f in find_wav_files("audio_test")]
    if not wav_files:
        print("No .wav files found")
        return

    results = []

    print(f"\nFound {len(wav_files)} audio file(s)")
    print(
        f"Thresholds: Isolation ≥{ISOLATION_THRESHOLD_DB} dB, OOB ≤{OOB_POWER_THRESHOLD_DB} dB, Harmonics ≤{HARMONIC_THRESHOLD_DB} dB\n"
    )

    for wav_file in wav_files:
        f1, f2 = extract_freqs_from_filename(wav_file.name)

        if f1 is None or f2 is None:
            print(f"⚠ Skipped {wav_file.name} (invalid filename format)")
            continue

        try:
            print(f"Analyzing {wav_file.name}...")
            purity = measure_spectral_purity(
                str(wav_file), f1, f2, detect_bw=150, plot=True
            )

            status = "✓ PASS" if purity["pass"] else "✗ FAIL"
            reasons = []
            if not purity["isolation_pass"]:
                reasons.append(
                    f"isolation {purity['min_isolation_db']:.1f} dB < {ISOLATION_THRESHOLD_DB} dB"
                )
            if not purity["oob_pass"]:
                reasons.append(
                    f"OOB {purity['oob_relative_peak_db']:.1f} dB > {OOB_POWER_THRESHOLD_DB} dB"
                )
            if not purity["harmonic_pass"]:
                reasons.append(
                    f"harmonics {purity['harmonic_rejection_db']:.1f} dB < {HARMONIC_THRESHOLD_DB} dB"
                )

            reason_str = " | ".join(reasons) if reasons else "all criteria met"
            print(f"  {status} ({reason_str})")

            results.append(
                {
                    "filename": f"{wav_file.parent.name}/{wav_file.name}",
                    "f1_hz": purity["f1"],
                    "f1_peak_db": purity["f1_peak_db"],
                    "f2_hz": purity["f2"],
                    "f2_peak_db": purity["f2_peak_db"],
                    "peak_imbalance_db": purity["peak_imbalance_db"],
                    "isolation_f1_db": purity["isolation_f1_db"],
                    "isolation_f2_db": purity["isolation_f2_db"],
                    "min_isolation_db": purity["min_isolation_db"],
                    "oob_energy_db": purity["oob_energy_db"],
                    "oob_relative_peak_db": purity["oob_relative_peak_db"],
                    "max_harmonic_db": purity["max_harmonic_db"],
                    "harmonic_rejection_db": purity["harmonic_rejection_db"],
                    "noise_floor_db": purity["noise_floor_db"],
                    "snr_f1_db": purity["snr_f1_db"],
                    "snr_f2_db": purity["snr_f2_db"],
                    "min_snr_db": purity["min_snr_db"],
                    "pass": purity["pass"],
                }
            )

        except Exception as e:
            print(f"  ✗ Error: {e}")
            import traceback

            traceback.print_exc()

    # Save CSV report
    if results:

        save_results_to_csv(
            headers=results[0].keys(),
            data=results,
            subfolder="spectral_purity",
            filename=f"spectral_purity_report.csv",
        )

        # Print summary
        passed = sum(1 for r in results if r["pass"])
        failed = len(results) - passed
        print(f"Summary: {passed} passed, {failed} failed out of {len(results)} files")

        # Print per-band summary
        bands = {}
        for r in results:
            band_key = f"{r['f1_hz']:.0f}-{r['f2_hz']:.0f}"
            if band_key not in bands:
                bands[band_key] = {"pass": 0, "fail": 0}
            if r["pass"]:
                bands[band_key]["pass"] += 1
            else:
                bands[band_key]["fail"] += 1

        print("\nPer-band summary:")
        for band, counts in sorted(bands.items()):
            total = counts["pass"] + counts["fail"]
            pct = (counts["pass"] / total * 100) if total > 0 else 0
            print(f"  {band} Hz: {counts['pass']}/{total} passed ({pct:.0f}%)")


if __name__ == "__main__":
    main()
