"""
Direction TDOA localization.

Walks test_files/direction/**/<subfolder>/ directories.
Mic coordinates are parsed from filenames: mic{N}_x{X}_y{Y}[_synced].wav
Localizes the dominant sound event in each subfolder using TDOA,
plots each subfolder individually and all results combined.

Run from Testing/ directory:
    python scripts/localize_tdoa.py
"""


import re
import os
import colorsys
from collections import defaultdict
import numpy as np
import soundfile as sf
from scipy.signal import butter, sosfilt
from scipy.optimize import least_squares
import matplotlib
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

matplotlib.use("macosx")
plt.rcParams.update(
    {
        "figure.facecolor": "#f8f9fa",
        "axes.facecolor": "#ffffff",
        "axes.edgecolor": "#cccccc",
        "axes.labelcolor": "#333333",
        "axes.titlesize": 13,
        "axes.labelsize": 11,
        "xtick.color": "#555555",
        "ytick.color": "#555555",
        "grid.color": "#e0e0e0",
        "grid.linewidth": 0.8,
        "legend.framealpha": 0.95,
        "legend.edgecolor": "#cccccc",
        "font.family": "sans-serif",
    }
)

# ── Config ────────────────────────────────────────────────────────────────────

DIRECTION_ROOT = "test_files/direction"
SPEED_OF_SOUND = 343.0  # m/s
MAX_TDOA_S = 15.0 / SPEED_OF_SOUND  # ~43 ms, generous for small rooms
BP_LOW = 500
BP_HIGH = 8000
EVENT_PRE_S = 0.4  # window before peak energy
EVENT_POST_S = 0.4  # window after peak energy
OUTPUT_DIR = "results/direction"

# ── Helpers ───────────────────────────────────────────────────────────────────

COORD_RE = re.compile(
    r"mic(\d+)_x(-?[0-9]+(?:\.[0-9]+)?)_y(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE
)


def parse_mic_info(path):
    m = COORD_RE.search(os.path.basename(path))
    if not m:
        return None
    return int(m.group(1)), float(m.group(2)), float(m.group(3))


def load_audio(path):
    data, sr = sf.read(path)
    if data.ndim > 1:
        data = data[:, 0]
    return data, sr


def bandpass(sig, sr, low, high):
    sos = butter(4, [low, high], btype="band", fs=sr, output="sos")
    return sosfilt(sos, sig)


def peak_time(sig, sr, smooth_ms=5):
    """Return time (s) of peak energy in sig."""
    env = np.abs(sig)
    k = max(1, int(sr * smooth_ms / 1000))
    env = np.convolve(env, np.ones(k) / k, mode="same")
    return int(np.argmax(env)) / sr


def gcc(ref, sig, sr):
    """Return (TDOA seconds, cc_curve, lag_times_s) via cross-correlation, constrained to ±MAX_TDOA_S."""
    n = len(ref) + len(sig)
    R = np.fft.rfft(ref, n=n) * np.conj(np.fft.rfft(sig, n=n))
    cc = np.fft.irfft(R)
    max_lag = int(MAX_TDOA_S * sr)
    fwd = np.arange(0, max_lag + 1)
    bwd = np.arange(n - max_lag, n)
    idx = np.concatenate([fwd, bwd])
    lags = np.concatenate([fwd, np.arange(-max_lag, 0)])
    peak = np.argmax(cc[idx])
    sort_order = np.argsort(lags)
    return lags[peak] / sr, cc[idx][sort_order], lags[sort_order] / sr


def localize_tdoa(tdoas, mic_pairs, mic_coords, speed=SPEED_OF_SOUND):
    """Nonlinear least-squares source localization from TDOAs (Levenberg-Marquardt)."""

    def residuals(pos):
        x, y = pos
        res = []
        for (i, j), dt in zip(mic_pairs, tdoas):
            xi, yi = mic_coords[i]
            xj, yj = mic_coords[j]
            di = np.sqrt((x - xi) ** 2 + (y - yi) ** 2)
            dj = np.sqrt((x - xj) ** 2 + (y - yj) ** 2)
            res.append((di - dj) / speed - dt)
        return np.array(res)

    cx = np.mean(mic_coords[:, 0])
    cy = np.mean(mic_coords[:, 1])
    span = np.ptp(mic_coords, axis=0).max() or 1.0

    best = None
    for dx in [-span, 0, span]:
        for dy in [-span, 0, span]:
            r = least_squares(
                residuals,
                [cx + dx, cy + dy],
                method="lm",
                ftol=1e-12,
                xtol=1e-6,
                gtol=1e-12,
                max_nfev=30000,
            )
            if best is None or r.cost < best.cost:
                best = r

    rms = np.sqrt(np.mean(residuals(best.x) ** 2)) * speed
    return best.x, rms


def collect_subfolders(root):
    """Yield (dirpath, display_name, group, wav_files) for every leaf with coord-named WAVs."""
    results = []
    for dirpath, _, filenames in os.walk(root):
        wavs = [
            os.path.join(dirpath, f)
            for f in sorted(filenames)
            if f.lower().endswith(".wav") and COORD_RE.search(f)
        ]
        if wavs:
            rel = os.path.relpath(dirpath, root)
            group = rel.split(os.sep)[0]
            raw = os.path.basename(dirpath)
            display = raw.replace("_", " ").title()
            results.append((dirpath, display, group, wavs))
    return sorted(results)


def location_shaded_colors(names):
    """
    For names like 'Clap 1-1', 'Clap 1-2', 'Clap 2-1', ..., assign the same
    base hue per location number and vary lightness per variant number.
    Returns a dict mapping name -> RGBA color.
    """
    loc_re = re.compile(r"(\d+)-(\d+)")
    parsed = {}
    for name in names:
        m = loc_re.search(name)
        parsed[name] = (int(m.group(1)), int(m.group(2))) if m else (hash(name) % 100, 0)

    unique_locs = sorted({v[0] for v in parsed.values()})
    n_locs = max(len(unique_locs), 1)
    cmap = plt.cm.tab10
    base_colors = {
        loc: mcolors.to_rgba(cmap(i / n_locs)) for i, loc in enumerate(unique_locs)
    }

    loc_to_names = defaultdict(list)
    for name, (loc, var) in parsed.items():
        loc_to_names[loc].append((var, name))

    result = {}
    for loc, var_names in loc_to_names.items():
        var_names_sorted = sorted(var_names)
        r, g, b, a = base_colors[loc]
        h, lum, s = colorsys.rgb_to_hls(r, g, b)
        n_vars = len(var_names_sorted)
        for k, (_var, name) in enumerate(var_names_sorted):
            if n_vars == 1:
                new_lum = lum
            else:
                new_lum = 0.35 + k * (0.65 - 0.35) / (n_vars - 1)
            nr, ng, nb = colorsys.hls_to_rgb(h, new_lum, s)
            result[name] = (nr, ng, nb, a)

    return result


# ── Main ──────────────────────────────────────────────────────────────────────

subfolders = collect_subfolders(DIRECTION_ROOT)
if not subfolders:
    print(f"No coordinate-named WAV files found under {DIRECTION_ROOT}")
    raise SystemExit(1)

print(f"Found {len(subfolders)} subfolder(s)")
os.makedirs(OUTPUT_DIR, exist_ok=True)

subfolder_colors = location_shaded_colors([name for _, name, _, _ in subfolders])

all_results = []  # (name, group, mic_coords_arr, mic_labels, pos, rms)

for dirpath, name, group, wav_files in subfolders:
    print(f"\n{'='*60}")
    print(f"Subfolder: {name}")

    mics = []
    for wf in wav_files:
        info = parse_mic_info(wf)
        if info:
            mic_n, mx, my = info
            mics.append((mic_n, mx, my, wf))
    mics.sort(key=lambda m: m[0])

    if len(mics) < 3:
        print(f"  Only {len(mics)} mic(s) with coordinates — need ≥3, skipping")
        all_results.append((name, group, np.empty((0, 2)), [], None, None))
        continue

    mic_labels = [f"ARU {m[0]}" for m in mics]
    mic_coords = np.array([(m[1], m[2]) for m in mics])
    files = [m[3] for m in mics]

    print(f"  Mics: {mic_labels}")
    for lbl, (mx, my), fp in zip(mic_labels, mic_coords, files):
        print(f"    {lbl}: ({mx}, {my}) m  ← {os.path.basename(fp)}")

    # Load, bandpass, trim to common length
    segments, sr_ref = [], None
    for fp in files:
        data, sr = load_audio(fp)
        data = bandpass(data, sr, BP_LOW, BP_HIGH)
        segments.append(data)
        sr_ref = sr
    min_len = min(len(s) for s in segments)
    segments = [s[:min_len] for s in segments]

    # Anchor to peak energy in loudest mic
    peak_amps = [np.max(np.abs(s)) for s in segments]
    anchor_idx = int(np.argmax(peak_amps))
    t_peak = peak_time(segments[anchor_idx], sr_ref)
    print(f"  Event anchor: {mic_labels[anchor_idx]} peak @ {t_peak:.3f}s")

    # Extract aligned window from all mics
    s0 = max(0, int((t_peak - EVENT_PRE_S) * sr_ref))
    s1 = min(min_len, int((t_peak + EVENT_POST_S) * sr_ref))
    win_segs = [s[s0:s1] for s in segments]
    win_min = min(len(w) for w in win_segs)
    win_segs = [w[:win_min] for w in win_segs]

    # GCC TDOAs for all pairs
    n_mics = len(win_segs)
    mic_pairs = [(i, j) for i in range(n_mics) for j in range(i + 1, n_mics)]
    tdoas, valid_pairs, cc_data = [], [], []
    for i, j in mic_pairs:
        dt, cc_curve, cc_lags = gcc(win_segs[i], win_segs[j], sr_ref)
        if abs(dt) <= MAX_TDOA_S:
            tdoas.append(dt)
            valid_pairs.append((i, j))
            cc_data.append((i, j, dt, cc_curve, cc_lags))
            print(f"  TDOA {mic_labels[i]} vs {mic_labels[j]}: {dt*1000:.3f} ms")
        else:
            cc_data.append((i, j, dt, cc_curve, cc_lags))
            print(
                f"  Dropping impossible TDOA {mic_labels[i]} vs {mic_labels[j]}: "
                f"{dt*1000:.1f} ms"
            )

    if len(valid_pairs) < 3:
        print(f"  Only {len(valid_pairs)} valid pair(s) — cannot localize")
        all_results.append((name, group, mic_coords, mic_labels, None, None))
        continue

    pos, rms = localize_tdoa(tdoas, valid_pairs, mic_coords)
    print(
        f"  → position: x={pos[0]:.3f} m, y={pos[1]:.3f} m  |  RMS residual: {rms:.4f} m"
    )
    all_results.append((name, group, mic_coords, mic_labels, pos, rms))

    # ── CC debug plot ─────────────────────────────────────────────────────────
    n_pairs = len(cc_data)
    cols = min(n_pairs, 3)
    rows = (n_pairs + cols - 1) // cols
    fig_cc, axes_cc = plt.subplots(
        rows, cols, figsize=(5 * cols, 3 * rows), squeeze=False
    )


    for idx, (i, j, dt, cc_curve, cc_lags) in enumerate(cc_data):
        ax_cc = axes_cc[idx // cols][idx % cols]
        lags_ms = cc_lags * 1000
        # Normalize so peak = 1 for easy comparison across pairs
        cc_norm = cc_curve / (np.max(np.abs(cc_curve)) + 1e-12)
        valid = abs(dt) <= MAX_TDOA_S

        ax_cc.fill_between(
            lags_ms, cc_norm, alpha=0.25, color="#2196F3" if valid else "#E53935"
        )
        ax_cc.plot(
            lags_ms, cc_norm, linewidth=0.9, color="#1565C0" if valid else "#B71C1C"
        )

        # Peak marker
        ax_cc.axvline(
            dt * 1000,
            color="#FF6F00",
            linewidth=1.4,
            linestyle="--",
            label=f"peak = {dt*1000:.3f} ms",
        )
        ax_cc.axvline(0, color="#888888", linewidth=0.7, linestyle=":")
        ax_cc.axhline(0, color="#888888", linewidth=0.5)

        pair_label = f"{mic_labels[i]} vs {mic_labels[j]}"
        status = "" if valid else "  [dropped]"
        ax_cc.set_title(
            f"{pair_label}{status}", fontsize=9, color="#1a1a1a" if valid else "#B71C1C"
        )
        ax_cc.set_xlabel("lag (ms)", fontsize=8)
        ax_cc.set_ylabel("norm. CC", fontsize=8)
        ax_cc.tick_params(labelsize=7)
        ax_cc.legend(fontsize=7, loc="upper right")
        ax_cc.grid(True)

    # Hide any unused subplots
    for idx in range(n_pairs, rows * cols):
        axes_cc[idx // cols][idx % cols].set_visible(False)

    fig_cc.tight_layout()
    safe = name.replace("/", "_").replace("\\", "_")
    cc_out = os.path.join(OUTPUT_DIR, f"cc_{safe}.png")
    plt.savefig(cc_out, dpi=150, bbox_inches="tight")
    print(f"  CC debug plot saved → {cc_out}")
    plt.close(fig_cc)

    # ── Per-subfolder plot ────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(7, 6))

    for lbl, (mx, my) in zip(mic_labels, mic_coords):
        ax.plot(
            mx,
            my,
            "^",
            color="#ff1a1a",
            markersize=13,
            markeredgecolor="#800000",
            markeredgewidth=1.0,
            zorder=4,
        )
        ax.annotate(
            lbl,
            (mx, my),
            xytext=(7, 5),
            textcoords="offset points",
            fontsize=9,
            fontweight="bold",
            color="#800000",
        )
        # Line from mic to estimated source
        ax.plot(
            [mx, pos[0]],
            [my, pos[1]],
            color="#aaaaaa",
            linewidth=0.8,
            linestyle="--",
            zorder=2,
        )

    src_color = subfolder_colors[name]

    # RMS error circle
    circle = mpatches.Circle(
        pos,
        rms,
        fill=False,
        edgecolor=src_color,
        linewidth=1.2,
        linestyle=":",
        zorder=4,
        label=f"RMS error = {rms:.3f} m",
    )
    ax.add_patch(circle)

    ax.scatter(
        pos[0],
        pos[1],
        s=180,
        color=src_color,
        zorder=5,
        edgecolors="white",
        linewidths=1.2,
        label=f"Source  ({pos[0]:.2f}, {pos[1]:.2f}) m",
    )

    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    # ax.set_title(f"TDOA localization — {name}", pad=10)
    ax.legend(fontsize=9, loc="best")
    ax.grid(True)

    safe = name.replace("/", "_").replace("\\", "_")
    out = os.path.join(OUTPUT_DIR, f"localization_{safe}.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  Plot saved → {out}")
    plt.close(fig)

# ── Combined plots — one per top-level subfolder ──────────────────────────────

groups_order = []
groups_map = {}
for entry in all_results:
    g = entry[1]
    if g not in groups_map:
        groups_map[g] = []
        groups_order.append(g)
    groups_map[g].append(entry)

for grp in groups_order:
    grp_results = groups_map[grp]

    fig, ax = plt.subplots(figsize=(9, 7))
    mic_drawn = set()

    for (name, _grp, mic_coords, mic_labels, pos, rms) in grp_results:
        color = subfolder_colors[name]
        if len(mic_coords) == 0:
            continue

        for lbl, (mx, my) in zip(mic_labels, mic_coords):
            key = (round(mx, 4), round(my, 4))
            if key not in mic_drawn:
                ax.plot(
                    mx,
                    my,
                    "^",
                    color="#ff1a1a",
                    markersize=13,
                    markeredgecolor="#800000",
                    markeredgewidth=1.0,
                    zorder=6,
                )
                ax.annotate(
                    lbl,
                    (mx, my),
                    xytext=(7, 5),
                    textcoords="offset points",
                    fontsize=9,
                    fontweight="bold",
                    color="#800000",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7),
                )
                mic_drawn.add(key)

        if pos is not None:
            circle = mpatches.Circle(
                pos,
                rms,
                fill=True,
                facecolor=color,
                alpha=0.12,
                edgecolor=color,
                linewidth=1.0,
                linestyle=":",
                zorder=3,
            )
            ax.add_patch(circle)
            ax.scatter(
                pos[0],
                pos[1],
                s=140,
                color=color,
                zorder=5,
                edgecolors="white",
                linewidths=1.0,
                label=name,
            )
        else:
            ax.scatter([], [], color=color, s=80, label=name)

    mic_proxy = plt.Line2D(
        [0],
        [0],
        marker="^",
        color="w",
        markerfacecolor="#ff1a1a",
        markeredgecolor="#800000",
        markersize=11,
        label="ARU",
    )
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(
        [mic_proxy] + handles,
        ["ARU"] + labels,
        bbox_to_anchor=(1.01, 1),
        loc="upper left",
        fontsize=8.5,
        framealpha=0.95,
    )

    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    fig.tight_layout()

    combined_out = os.path.join(OUTPUT_DIR, f"localization_combined_{grp}.png")
    plt.savefig(combined_out, dpi=150, bbox_inches="tight")
    print(f"\nCombined plot saved → {combined_out}")

plt.show()
