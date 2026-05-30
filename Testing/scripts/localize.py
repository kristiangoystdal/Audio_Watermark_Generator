"""
Acoustic localization of bird detections from subfolders of
test_files/acoustic_localization/.

Run from the Testing/ directory:
    python scripts/localize.py

Each subfolder that contains both aru_coords.csv and detections.csv is
processed independently and produces one result plot.

Calibration lookup order per subfolder:
  1. <subfolder>/calibration.json
  2. test_files/acoustic_localization/calibration.json  (shared fallback)
  If neither exists, a calibration GUI opens and saves to the shared path.

To redo calibration: delete the relevant calibration.json and rerun.
"""

import warnings
from helper.helper import save_plot_to_results_folder
warnings.filterwarnings("ignore")

import json, os
import pandas as pd
import numpy as np
import pytz
from datetime import datetime, timedelta
from collections import defaultdict
from scipy.linalg import lstsq

import matplotlib
matplotlib.use("macosx")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm
import mplcursors

from opensoundscape.localization import SynchronizedRecorderArray

# ── Shared configuration ──────────────────────────────────────────────────────

ACOUSTIC_LOC_DIR = "test_files/acoustic_localization"

MAX_RESIDUAL      = 10
AREA_LIMIT        = 300
MIN_N_RECEIVERS   = 4
MAX_RECEIVER_DIST = 100
CC_FILTER         = "phat"
CC_THRESHOLD      = 0.01

BANDPASS_RANGES = {
    "Bank Swallow":              [3000, 8000],
    "Bar-tailed Godwit":         [1000, 4000],
    "Barn Swallow":              [2000, 8000],
    "Black Redstart":            [2000, 8000],
    "Black Woodpecker":          [1000, 4000],
    "Bluethroat":                [2000, 9000],
    "Bohemian Waxwing":          [5000, 9000],
    "Canada Goose":              [ 500, 2000],
    "Coal Tit":                  [6000, 11000],
    "Common Buzzard":            [ 500, 2000],
    "Common Chaffinch":          [2000, 8000],
    "Common Chiffchaff":         [4000, 8000],
    "Common Gull":               [ 500, 2500],
    "Common House-Martin":       [3000, 7000],
    "Common Loon":               [ 500, 2500],
    "Common Redpoll":            [2000, 7000],
    "Common Redstart":           [3000, 9000],
    "Common Rosefinch":          [2000, 7000],
    "Common Scoter":             [ 500, 2000],
    "Common Swift":              [3000, 8000],
    "Dunlin":                    [2000, 6000],
    "Dunnock":                   [2000, 8000],
    "Eurasian Blackbird":        [2000, 7000],
    "Eurasian Blackcap":         [2000, 8000],
    "Eurasian Blue Tit":         [5000, 12000],
    "Eurasian Coot":             [ 500, 2000],
    "Eurasian Dotterel":         [2000, 5000],
    "Eurasian Jay":              [1000, 5000],
    "Eurasian Moorhen":          [ 500, 3000],
    "Eurasian Nuthatch":         [2000, 8000],
    "Eurasian Pygmy-Owl":        [1000, 4000],
    "Eurasian Siskin":           [3000, 9000],
    "Eurasian Sparrowhawk":      [2000, 6000],
    "Eurasian Treecreeper":      [5000, 9000],
    "Eurasian Woodcock":         [1000, 5000],
    "Eurasian Wren":             [3000, 9000],
    "European Greenfinch":       [2000, 7000],
    "European Pied Flycatcher":  [2000, 8000],
    "European Robin":            [2000, 8000],
    "European Stonechat":        [3000, 8000],
    "Fieldfare":                 [1000, 4000],
    "Goldcrest":                 [6000, 11000],
    "Gray Heron":                [ 500, 2000],
    "Gray Wagtail":              [3000, 7000],
    "Great Spotted Woodpecker":  [2000, 6000],
    "Great Tit":                 [3000, 9000],
    "Herring Gull":              [ 500, 2500],
    "Lesser Black-backed Gull":  [ 500, 2500],
    "Lesser Spotted Woodpecker": [3000, 8000],
    "Little Ringed Plover":      [2000, 6000],
    "Marsh Tit":                 [3000, 8000],
    "Mistle Thrush":             [2000, 7000],
    "Northern Goshawk":          [1000, 5000],
    "Northern Lapwing":          [1000, 4000],
    "Ortolan Bunting":           [2000, 7000],
    "Osprey":                    [1000, 4000],
    "Pine Grosbeak":             [2000, 7000],
    "Red-breasted Flycatcher":   [3000, 8000],
    "Redwing":                   [2000, 8000],
    "Reed Bunting":              [2000, 6000],
    "Ring Ouzel":                [2000, 7000],
    "Ring-necked Pheasant":      [ 500, 2000],
    "Ruddy Turnstone":           [2000, 5000],
    "Song Thrush":               [2000, 8000],
    "Spotted Flycatcher":        [3000, 8000],
    "Tawny Owl":                 [ 500, 2000],
    "Tree Pipit":                [2000, 8000],
    "Whooper Swan":              [ 500, 2000],
    "Wood Lark":                 [2000, 8000],
    "Wood Sandpiper":            [2000, 6000],
    "Wood Warbler":              [3000, 9000],
    "Yellowhammer":              [2000, 7000],
}

# ── Calibration helpers ───────────────────────────────────────────────────────

def run_calibration(image, mic_xy_m, mic_labels, save_path):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(image)
    ax.set_title(
        "CALIBRATION  —  click on each microphone in order.\n"
        "mic1 first, then mic2, mic3, mic4.\n"
        "Press Enter or close the window when done.",
        fontsize=11, pad=10,
    )
    ax.axis("off")

    clicked_px  = []
    annotations = []
    prompt_colors = ["#e74c3c", "#e67e22", "#27ae60", "#2980b9"]

    def update_title():
        n = len(clicked_px)
        if n < 4:
            ax.set_title(
                f"CALIBRATION  —  click {n+1}/4: where is  {mic_labels[n]}  on the image?\n"
                f"({mic_labels[n]} is at metre coords "
                f"({mic_xy_m[n,0]:.1f}, {mic_xy_m[n,1]:.1f}))\n"
                "Press Backspace to undo last click.",
                fontsize=11, color=prompt_colors[n], pad=10,
            )
        else:
            ax.set_title(
                "All 4 microphones marked.\n"
                "Close this window to continue  (or press Backspace to undo).",
                fontsize=11, color="green", pad=10,
            )
        fig.canvas.draw_idle()

    update_title()

    def on_click(event):
        if event.inaxes is not ax or event.button != 1:
            return
        if len(clicked_px) >= 4:
            return
        px, py = event.xdata, event.ydata
        clicked_px.append([px, py])
        n = len(clicked_px)
        color = prompt_colors[n - 1]
        dot, = ax.plot(px, py, "o", color=color, markersize=10, zorder=5)
        txt  = ax.annotate(
            mic_labels[n - 1], (px, py),
            xytext=(6, 6), textcoords="offset points",
            fontsize=10, fontweight="bold", color=color, zorder=6,
        )
        annotations.append((dot, txt))
        update_title()

    def on_key(event):
        if event.key == "backspace" and clicked_px:
            clicked_px.pop()
            dot, txt = annotations.pop()
            dot.remove()
            txt.remove()
            update_title()

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.tight_layout()
    plt.show()

    if len(clicked_px) < 4:
        raise RuntimeError(
            f"Only {len(clicked_px)} mics calibrated — need 4.  Rerun to try again."
        )

    pixels = np.array(clicked_px)
    with open(save_path, "w") as f:
        json.dump({"pixels": pixels.tolist(), "mic_labels": mic_labels}, f, indent=2)
    print(f"Calibration saved to {save_path}")
    return pixels


def load_calibration(path):
    with open(path) as f:
        data = json.load(f)
    return np.array(data["pixels"])


def build_transform(mic_xy_m, mic_px):
    A  = np.column_stack([mic_xy_m, np.ones(len(mic_xy_m))])
    cx, *_ = lstsq(A, mic_px[:, 0])
    cy, *_ = lstsq(A, mic_px[:, 1])

    def m2px(xm, ym):
        xm, ym = np.asarray(xm), np.asarray(ym)
        return (cx[0]*xm + cx[1]*ym + cx[2],
                cy[0]*xm + cy[1]*ym + cy[2])
    return m2px


# ── Per-subfolder processing ──────────────────────────────────────────────────

def process_subfolder(subfolder_path, subfolder_name):
    print(f"\n{'='*60}")
    print(f"Processing: {subfolder_name}")
    print(f"{'='*60}")

    aru_csv        = os.path.join(subfolder_path, "aru_coords.csv")
    detections_csv = os.path.join(subfolder_path, "detections.csv")
    local_calib    = os.path.join(subfolder_path, "calibration.json")

    image        = mpimg.imread(os.path.join(subfolder_path, "GPS.png"))
    img_h, img_w = image.shape[:2]

    aru_coords = pd.read_csv(aru_csv, index_col=0)
    aru_coords.index = [os.path.abspath(idx) for idx in aru_coords.index]
    mic_xy_m    = aru_coords[["x", "y"]].values
    mic_labels  = [idx.split("/")[-1].split("_")[0] for idx in aru_coords.index]

    if os.path.exists(local_calib):
        print(f"Loading calibration from {local_calib}")
        mic_px = load_calibration(local_calib)
    else:
        print("No calibration found — starting calibration GUI …")
        mic_px = run_calibration(image, mic_xy_m, mic_labels, local_calib)

    m2px = build_transform(mic_xy_m, mic_px)

    print("Loading detections …")
    detections = pd.read_csv(detections_csv)
    print(f"  [debug] detections file sample: {detections['file'].iloc[0]!r}")
    print(f"  [debug] aru_coords index sample: {aru_coords.index[0]!r}")
    # detections paths are missing the subfolder segment; insert it and resolve absolute
    detections["file"] = detections["file"].apply(
        lambda p: os.path.abspath(
            p.replace(ACOUSTIC_LOC_DIR + "/", subfolder_path + "/", 1)
            if ("/" + subfolder_name + "/") not in p
            else p
        )
    )
    base_time  = pytz.timezone("Europe/Oslo").localize(datetime(2024, 1, 1, 12, 0, 0))
    detections["start_timestamp"] = [
        base_time + timedelta(seconds=s) for s in detections["start_time"]
    ]
    detections = detections.set_index(["file", "start_time", "end_time", "start_timestamp"])

    print("Initialising recorder array …")
    array = SynchronizedRecorderArray(aru_coords)

    print("Running localization …")
    position_estimates = array.localize_detections(
        detections,
        min_n_receivers=MIN_N_RECEIVERS,
        max_receiver_dist=MAX_RECEIVER_DIST,
        localization_algorithm="least_squares",
        cc_filter=CC_FILTER,
        cc_threshold=CC_THRESHOLD,
        residual_threshold=MAX_RESIDUAL,
        bandpass_ranges=BANDPASS_RANGES,
    )

    filtered = [
        e for e in position_estimates
        if abs(e.location_estimate[0]) < AREA_LIMIT
        and abs(e.location_estimate[1]) < AREA_LIMIT
    ]

    by_species    = defaultdict(list)
    for e in filtered:
        by_species[e.class_name].append(e)
    sorted_species = sorted(by_species.keys())
    print(f"  → {len(filtered)} estimates · {len(sorted_species)} species: {sorted_species}")

    # ── Plot ──────────────────────────────────────────────────────────────────

    cmap = cm.get_cmap("tab20", max(len(sorted_species), 1))

    fig, ax = plt.subplots(figsize=(11, 10))
    fig.subplots_adjust(right=0.70)

    ax.imshow(image, extent=[0, img_w, img_h, 0], zorder=0)
    ax.set_xlim(0, img_w)
    ax.set_ylim(img_h, 0)

    scatter_handles = []
    point_meta      = {}

    for i, sp in enumerate(sorted_species):
        estimates = by_species[sp]
        xm = [e.location_estimate[0] for e in estimates]
        ym = [e.location_estimate[1] for e in estimates]
        px, py = m2px(xm, ym)
        rms    = [e.residual_rms for e in estimates]
        ts     = [e.start_timestamp.strftime("%H:%M:%S") for e in estimates]

        sc = ax.scatter(px, py,
                        color=cmap(i), s=70, alpha=0.85, zorder=3,
                        label=f"{sp}  (n={len(estimates)})",
                        edgecolors="white", linewidths=0.5,
                        picker=True)
        scatter_handles.append(sc)
        point_meta[sc] = list(zip([sp]*len(xm), xm, ym, rms, ts))

    for label, (xm, ym), (px, py) in zip(
            mic_labels,
            mic_xy_m,
            zip(*m2px(mic_xy_m[:, 0], mic_xy_m[:, 1]))):
        ax.plot(px, py, "^", color="white", markersize=10,
                markeredgecolor="black", markeredgewidth=1.2, zorder=5)
        ax.annotate(label, (px, py), xytext=(5, -12),
                    textcoords="offset points", fontsize=9,
                    fontweight="bold", color="white",
                    path_effects=[
                        __import__("matplotlib.patheffects", fromlist=["withStroke"])
                        .withStroke(linewidth=2, foreground="black")
                    ],
                    zorder=6)

    ax.set_title(
        f"{subfolder_name}  —  {len(filtered)} detections · {len(sorted_species)} species\n"
        f"residual < {MAX_RESIDUAL} m  |  hover for details  |  click legend to toggle",
        fontsize=10,
    )
    ax.axis("off")

    leg = ax.legend(
        handles=scatter_handles,
        bbox_to_anchor=(1.01, 1), loc="upper left",
        fontsize=8, framealpha=0.92,
        title="Species  (click to toggle)",
        title_fontsize=8,
    )

    leg_map = {}
    for leg_artist, sc in zip(leg.legend_handles, scatter_handles):
        leg_artist.set_picker(True)
        leg_artist.set_pickradius(6)
        leg_map[leg_artist] = sc

    def on_pick(event):
        if event.artist in leg_map:
            sc      = leg_map[event.artist]
            visible = not sc.get_visible()
            sc.set_visible(visible)
            event.artist.set_alpha(0.9 if visible else 0.2)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("pick_event", on_pick)

    cursor = mplcursors.cursor(scatter_handles, hover=True)

    @cursor.connect("add")
    def on_hover(sel):
        sp, xm, ym, rms, ts = point_meta[sel.artist][sel.index]
        sel.annotation.set_text(
            f"{sp}\nx = {xm:.1f} m,  y = {ym:.1f} m\n"
            f"residual = {rms:.2f} m\ntime = {ts}"
        )
        sel.annotation.get_bbox_patch().set(fc="white", alpha=0.92)

    plot_filename = f"localization_{subfolder_name}.png"
    save_plot_to_results_folder(fig, "acoustic_localization", plot_filename, dpi=300)
    plt.close(fig)


# ── Discover and process all subfolders ───────────────────────────────────────

subfolders = sorted(
    entry.name for entry in os.scandir(ACOUSTIC_LOC_DIR)
    if entry.is_dir()
    and os.path.exists(os.path.join(entry.path, "aru_coords.csv"))
    and os.path.exists(os.path.join(entry.path, "detections.csv"))
)

if not subfolders:
    print(f"No subfolders with aru_coords.csv + detections.csv found in {ACOUSTIC_LOC_DIR}/")
else:
    print(f"Found {len(subfolders)} subfolder(s) to process: {subfolders}")
    for name in subfolders:
        path = os.path.join(ACOUSTIC_LOC_DIR, name)
        process_subfolder(path, name)

print("\nDone.")
