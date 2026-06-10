import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator
from pathlib import Path

from helper.helper import save_plot_to_results_folder

MEASURES_DIR = Path(__file__).parent.parent / "test_files" / "temperature_range"
SUBFOLDER = "temperature_range"

# ── Parse label files ────────────────────────────────────────────────────────

PATTERN_FULL = re.compile(
    r"^\d+\.\d+\t\d+\.\d+\t"
    r"Message ID: (\d+) \| "
    r"Time: (\d{2}):(\d{2}):(\d{2}) on \w+ \d{2}/\d{2}/\d{4} \| "
    r"Message: \w+ \| Device ID: \d+ \| "
    r"Location: [\d.]+,[\d.]+ \| "
    r"Temperature: (-?\d+)°C$"
)

PATTERN_GARBLED = re.compile(r"[^\x09\x0a\x0d\x20-\x7e]")

print(f"[DEBUG] Input directory : {MEASURES_DIR}")
print(f"[DEBUG] Directory exists: {MEASURES_DIR.exists()}")

txt_files = sorted(
    f for f in MEASURES_DIR.glob("*.txt") if "debug" not in f.stem.lower()
)
print(f"[DEBUG] Found {len(txt_files)} .txt file(s):")
for f in txt_files:
    print(f"[DEBUG]   {f.name}  ({f.stat().st_size} bytes)")

if not txt_files:
    print(f"No .txt files found in {MEASURES_DIR}")
    exit(1)

records = []
garbled_count = 0
seen = set()
duplicate_count = 0
no_match_count = 0

for filepath in txt_files:
    file_records = 0
    file_garbled = 0
    file_duplicates = 0
    file_no_match = 0
    print(f"[DEBUG] Parsing: {filepath.name}")
    with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            m = PATTERN_FULL.match(line)
            if m:
                h, mn, s = int(m.group(2)), int(m.group(3)), int(m.group(4))
                t = h * 3600 + mn * 60 + s
                if t not in seen:
                    seen.add(t)
                    records.append((t, int(m.group(5))))
                    file_records += 1
                else:
                    file_duplicates += 1
                    duplicate_count += 1
                continue
            if PATTERN_GARBLED.search(line):
                garbled_count += 1
                file_garbled += 1
            else:
                no_match_count += 1
                file_no_match += 1
    print(
        f"[DEBUG]   → decoded: {file_records}  duplicates: {file_duplicates}"
        f"  garbled: {file_garbled}  unmatched: {file_no_match}"
    )

print(f"[DEBUG] Total records   : {len(records)}")
print(f"[DEBUG] Total duplicates: {duplicate_count}")
print(f"[DEBUG] Total garbled   : {garbled_count}")
print(f"[DEBUG] Total unmatched : {no_match_count}")

records.sort()
t_arr = np.array([r[0] for r in records])
temp_arr = np.array([r[1] for r in records])

# ── Statistics ───────────────────────────────────────────────────────────────

print(
    f"[DEBUG] Time range: {int(t_arr[0]//3600):02d}:{int((t_arr[0]%3600)//60):02d}"
    f" → {int(t_arr[-1]//3600):02d}:{int((t_arr[-1]%3600)//60):02d}"
    f"  ({(t_arr[-1]-t_arr[0])/60:.1f} min)"
)
print(f"[DEBUG] Temp range: {temp_arr.min()} °C to {temp_arr.max()} °C")

intervals = np.diff(t_arr)
MSG_INTERVAL = float(np.median(intervals[intervals < intervals.mean() * 2]))
expected = round((t_arr[-1] - t_arr[0]) / MSG_INTERVAL) + 1
decoded = len(records)

print(f"[DEBUG] Median message interval: {MSG_INTERVAL:.1f}s")
print(
    f"[DEBUG] Expected messages: {expected}  Decoded: {decoded}"
    f"  ({decoded/expected*100:.1f}%)"
)

GAP_THRESHOLD = MSG_INTERVAL * 1.5
gap_mask = np.diff(t_arr) > GAP_THRESHOLD
gap_starts = t_arr[:-1][gap_mask]
gap_ends = t_arr[1:][gap_mask]
gap_missed = np.round((gap_ends - gap_starts) / MSG_INTERVAL).astype(int) - 1

print(f"[DEBUG] Gap threshold: {GAP_THRESHOLD:.1f}s  Gaps found: {len(gap_starts)}")
for i, (gs, ge, nm) in enumerate(zip(gap_starts, gap_ends, gap_missed)):
    print(
        f"[DEBUG]   Gap {i}: {int(gs//3600):02d}:{int((gs%3600)//60):02d}"
        f" → {int(ge//3600):02d}:{int((ge%3600)//60):02d}"
        f"  ({(ge-gs)/60:.1f} min, {nm} missed)"
    )

# ── Reference: baseline → cold ramp transition (data-driven) ─────────────────

BASELINE_MAX = 26
BASELINE_MIN = 20
BASELINE_MIN_N = 5

baseline_confirmed = False
baseline_count = 0
t0 = None

for t, temp in zip(t_arr, temp_arr):
    if not baseline_confirmed:
        if BASELINE_MIN <= temp <= BASELINE_MAX:
            baseline_count += 1
            if baseline_count >= BASELINE_MIN_N:
                baseline_confirmed = True
        else:
            baseline_count = 0
    else:
        if temp < BASELINE_MIN:
            t0 = t
            break

if t0 is None:
    raise RuntimeError("Could not detect baseline → cold ramp transition in data")

print(
    f"[DEBUG] Baseline confirmed after {baseline_count} samples"
    f" in [{BASELINE_MIN}, {BASELINE_MAX}] °C range"
)
print(
    f"[DEBUG] t0 (baseline→ramp): {int(t0//3600):02d}:{int((t0%3600)//60):02d}"
    f"  ({t0}s from midnight)"
)

# ── Procedure-derived stage boundaries (anchored to t0) ──────────────────────

stages_raw = [
    # (strip label, target label, t_start_s, t_end_s, ramp_end_s | None)
    ("Baseline", "+23 °C", int(t_arr[0]), t0, None),
    ("Cold\nramp", "→ −20 °C", t0, t0 + 43 * 60, None),
    ("Cold soak", "−20 °C", t0 + 43 * 60, t0 + 133 * 60, None),
    ("Step 1", "→ −5 °C", t0 + 133 * 60, t0 + 193 * 60, t0 + 148 * 60),
    ("Step 2", "→ +10 °C", t0 + 193 * 60, t0 + 253 * 60, t0 + 208 * 60),
    ("Step 3", "→ +25 °C", t0 + 253 * 60, t0 + 313 * 60, t0 + 268 * 60),
    ("Step 4", "→ +40 °C", t0 + 313 * 60, t0 + 373 * 60, t0 + 328 * 60),
    ("Step 5", "→ +55 °C", t0 + 373 * 60, t0 + 433 * 60, t0 + 388 * 60),
    ("Hot soak", "+65 °C", t0 + 433 * 60, t0 + 533 * 60, t0 + 443 * 60),
    ("Return", "→ +23 °C", t0 + 533 * 60, int(t_arr[-1]), t0 + 575 * 60),
]

STAGE_COLORS = [
    "#EEF3F8",  # Baseline
    "#E4EDF5",  # Cold ramp
    "#D8E6F2",  # Cold soak
    "#EAF0EA",  # Step 1
    "#EDF2E8",  # Step 2
    "#F2F3E8",  # Step 3
    "#F5EEE4",  # Step 4
    "#F5E8E0",  # Step 5
    "#F2E0D8",  # Hot soak
    "#EEF3F8",  # Return
]

# ── Per-stage dwell coverage (for thesis table) ───────────────────────────────
# Dwell window = ramp_end → stage_end  (or full stage if no ramp)

print(f"\n[TABLE] Per-stage dwell coverage:")
print(f"{'Stage':<12} {'Target':<10} {'Dwell':<16} {'Dec':>5} {'Exp':>5} {'Rate':>7}")
print("-" * 58)

table_rows = []
for label, target, ts, te, ramp_end in stages_raw:
    dwell_s = ramp_end if ramp_end is not None else ts
    dwell_e = te
    dwell_mask = (t_arr >= dwell_s) & (t_arr < dwell_e)
    t_dwell = t_arr[dwell_mask]

    n_decoded = len(t_dwell)
    duration = dwell_e - dwell_s
    n_expected = round(duration / MSG_INTERVAL) if duration > 0 else 0
    rate = n_decoded / n_expected * 100 if n_expected > 0 else 0.0

    # gaps within dwell
    dwell_gaps = []
    if len(t_dwell) > 1:
        diffs = np.diff(t_dwell)
        for j, d in enumerate(diffs):
            if d > GAP_THRESHOLD:
                dwell_gaps.append(int(np.round(d / MSG_INTERVAL) - 1))

    dwell_str = (
        f"{int(dwell_s//3600):02d}:{int((dwell_s%3600)//60):02d}"
        f"–{int(dwell_e//3600):02d}:{int((dwell_e%3600)//60):02d}"
    )
    label_clean = label.replace("\n", " ")
    flag = "*" if dwell_gaps else ""
    print(
        f"{label_clean:<12} {target:<10} {dwell_str:<16}"
        f" {n_decoded:>5} {n_expected:>5} {rate:>6.1f}%{flag}"
    )
    table_rows.append((label_clean, target, n_decoded, n_expected, rate, dwell_gaps))

print()
for label_clean, target, n_dec, n_exp, rate, dwell_gaps in table_rows:
    if dwell_gaps:
        print(
            f"[TABLE]   * {label_clean} ({target}): gaps with"
            f" {dwell_gaps} missed message(s) in dwell"
        )

# ── Null-broken trace (hours) ────────────────────────────────────────────────

plot_t, plot_temp = [], []
for i, (t, temp) in enumerate(zip(t_arr, temp_arr)):
    if i > 0 and (t - t_arr[i - 1]) > GAP_THRESHOLD:
        plot_t.append(np.nan)
        plot_temp.append(np.nan)
    plot_t.append(t / 3600)
    plot_temp.append(temp)

# ── Plot ─────────────────────────────────────────────────────────────────────

YMIN, YMAX = -25, 72
BLUE = "#2166AC"
GAP_COLOR = "#FF0000"

fig = plt.figure(figsize=(14, 5.8))
gs_layout = fig.add_gridspec(2, 1, height_ratios=[1, 8], hspace=0.0)
ax_strip = fig.add_subplot(gs_layout[0])
ax = fig.add_subplot(gs_layout[1])

x_min = t_arr[0] / 3600 - 0.05
x_max = t_arr[-1] / 3600 + 0.12

# Stage label strip
for (label, sublabel, ts, te, _), fc in zip(stages_raw, STAGE_COLORS):
    ax_strip.axvspan(
        ts / 3600, te / 3600, facecolor=fc, edgecolor="#C0C8D0", linewidth=0.5
    )
    mid = (ts + te) / 2 / 3600
    stage_pts = (
        (te - ts) / 3600 / (x_max - x_min) * (fig.get_figwidth() - 1.0) * 72 * 0.85
    )
    fs_label = min(13.5, stage_pts / (len(label) * 0.55))
    fs_sub = min(12.5, stage_pts / (len(sublabel) * 0.55))
    ax_strip.text(
        mid,
        0.74,
        label,
        ha="center",
        va="center",
        fontsize=fs_label,
        fontweight="bold",
        color="#222",
        clip_on=True,
    )
    ax_strip.text(
        mid,
        0.24,
        sublabel,
        ha="center",
        va="center",
        fontsize=fs_sub,
        color="#555",
        clip_on=True,
    )
ax_strip.set_xlim(x_min, x_max)
ax_strip.set_ylim(0, 1)
ax_strip.axis("off")

# Stage bands + ramp/dwell dividers
for (label, sublabel, ts, te, ramp_end), fc in zip(stages_raw, STAGE_COLORS):
    ax.axvspan(
        ts / 3600, te / 3600, facecolor=fc, edgecolor="#C0C8D0", linewidth=0.4, zorder=0
    )
    if ramp_end is not None:
        ax.axvline(
            ramp_end / 3600,
            color="#AABBCC",
            linewidth=0.9,
            linestyle=(0, (4, 3)),
            zorder=1,
        )

# Reception gaps
GAP_LABEL_BASE_Y = (YMIN + (-15)) / 2
GAP_LABEL_OFFSETS = [0, 12, 0, 12, 24, 36]

for i, (gs_t, ge_t, nm) in enumerate(zip(gap_starts, gap_ends, gap_missed)):
    ax.axvspan(
        gs_t / 3600,
        ge_t / 3600,
        facecolor=GAP_COLOR,
        alpha=0.30,
        zorder=2,
        linewidth=0,
    )
    mid_h = (gs_t + ge_t) / 2 / 3600
    t_start_str = f"{int(gs_t//3600):02d}:{int((gs_t%3600)//60):02d}"
    t_end_str = f"{int(ge_t//3600):02d}:{int((ge_t%3600)//60):02d}"
    label_str = f"{t_start_str}–{t_end_str}\n{nm} missed"
    offset = GAP_LABEL_OFFSETS[i] if i < len(GAP_LABEL_OFFSETS) else 0

# Temperature trace
ax.plot(plot_t, plot_temp, color=BLUE, linewidth=1.4, zorder=3, solid_capstyle="round")

# Axes
ax.set_xlim(x_min, x_max)
ax.set_ylim(YMIN, YMAX)
ax.set_ylabel("Temperature (°C)", fontsize=16)
ax.set_xlabel("Time of day (HH:MM)", fontsize=16)

tick_hours = np.arange(np.ceil(t_arr[0] / 3600), np.floor(t_arr[-1] / 3600) + 1)
ax.set_xticks(tick_hours)
ax.set_xticklabels(
    [f"{int(h):02d}:00" for h in tick_hours],
    rotation=45,
    ha="right",
    fontsize=15,
)
ax.yaxis.set_major_locator(MultipleLocator(10))
ax.tick_params(axis="y", labelsize=15)
ax.grid(axis="y", linewidth=0.4, alpha=0.35, linestyle=":", color="#999")
ax.grid(axis="x", linewidth=0.3, alpha=0.20, linestyle=":", color="#999")
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

# Legend
legend_elements = [
    Line2D([0], [0], color=BLUE, linewidth=1.5, label="Decoded temperature"),
    mpatches.Patch(facecolor=GAP_COLOR, alpha=0.25, label="Reception gap"),
    Line2D(
        [0],
        [0],
        color="#AABBCC",
        linewidth=0.9,
        linestyle=(0, (4, 3)),
        label="Ramp / dwell transition",
    ),
]
ax.legend(
    handles=legend_elements,
    loc="lower right",
    fontsize=15,
    framealpha=0.93,
    edgecolor="#ccc",
    borderpad=0.6,
    bbox_to_anchor=(0.995, 0.18),
)

print(
    f"[DEBUG] Stage boundaries (anchored to t0={int(t0//3600):02d}:{int((t0%3600)//60):02d}):"
)
for label, sublabel, ts, te, ramp_end in stages_raw:
    ramp_str = (
        f"  ramp→dwell at {int(ramp_end//3600):02d}:{int((ramp_end%3600)//60):02d}"
        if ramp_end
        else ""
    )
    print(
        f"[DEBUG]   {label.replace(chr(10),' '):<12} {sublabel:<10}"
        f"  {int(ts//3600):02d}:{int((ts%3600)//60):02d}"
        f" – {int(te//3600):02d}:{int((te%3600)//60):02d}{ramp_str}"
    )

plt.tight_layout(pad=0.5)
save_plot_to_results_folder(fig, SUBFOLDER, "temperature_range.png", dpi=300)
print(
    f"Saved.\n"
    f"  Reference t0 (baseline→ramp): "
    f"{t0 // 3600:02d}:{(t0 % 3600) // 60:02d}\n"
    f"  Decoded: {decoded}/{expected} ({decoded/expected*100:.1f}%)\n"
    f"  Median interval: {MSG_INTERVAL:.1f}s   Garbled: {garbled_count}\n"
    f"  Gaps: {len(gap_starts)}"
)
