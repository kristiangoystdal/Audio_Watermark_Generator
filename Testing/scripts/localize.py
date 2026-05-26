"""
Acoustic localization of bird detections from my_files/.

Run from the acoustic_localization/ directory:
    python localize.py

First run: a calibration GUI opens — click on each microphone's position in the
satellite image in order (mic1 → mic2 → mic3 → mic4).  The calibration is saved
to calibration.json and reused on every subsequent run.

To redo calibration: delete calibration.json and rerun.
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
from scipy.spatial import ConvexHull
from scipy.linalg import lstsq

import matplotlib
matplotlib.use("macosx")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as mpatches
import matplotlib.cm as cm
import mplcursors

from opensoundscape.localization import SynchronizedRecorderArray

# ── Configuration ────────────────────────────────────────────────────────────

MAP_IMAGE         = "test_files/acoustic_localization/GPS.png"
CALIBRATION_FILE  = "test_files/acoustic_localization/calibration.json"

MAX_RESIDUAL      = 10     # keep only estimates with residual_rms < this (metres)
AREA_LIMIT        = 300   # discard estimates that diverged beyond this (metres)
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

# ── Load receiver positions ───────────────────────────────────────────────────

aru_coords  = pd.read_csv("test_files/acoustic_localization/aru_coords.csv", index_col=0)
mic_xy_m    = aru_coords[["x", "y"]].values          # metres, mic1 = (0,0)
mic_labels  = [idx.split("/")[-1].split("_")[0] for idx in aru_coords.index]
image       = mpimg.imread(MAP_IMAGE)
img_h, img_w = image.shape[:2]

# ── Calibration ───────────────────────────────────────────────────────────────

def run_calibration():
    """
    Show the satellite image and ask the user to click on each microphone
    position in order.  Returns a (4,2) array of pixel coordinates.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(image)
    ax.set_title("CALIBRATION  —  click on each microphone in order.\n"
                 "mic1 first, then mic2, mic3, mic4.\n"
                 "Press Enter or close the window when done.",
                 fontsize=11, pad=10)
    ax.axis("off")

    clicked_px   = []          # pixel coords collected so far
    annotations  = []

    prompt_colors = ["#e74c3c", "#e67e22", "#27ae60", "#2980b9"]

    def update_title():
        n = len(clicked_px)
        if n < 4:
            next_label = mic_labels[n]
            ax.set_title(
                f"CALIBRATION  —  click {n+1}/4: where is  {next_label}  on the image?\n"
                f"({next_label} is at metre coords "
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
    with open(CALIBRATION_FILE, "w") as f:
        json.dump({"pixels": pixels.tolist(), "mic_labels": mic_labels}, f, indent=2)
    print(f"Calibration saved to {CALIBRATION_FILE}")
    return pixels


def load_calibration():
    with open(CALIBRATION_FILE) as f:
        data = json.load(f)
    return np.array(data["pixels"])


# ── Affine transform: metres → image pixels ───────────────────────────────────

def build_transform(mic_xy_m, mic_px):
    """
    Least-squares affine transform from metre coords to image pixel coords.
    mic_xy_m : (N,2) float  — metre positions of calibration points
    mic_px   : (N,2) float  — corresponding pixel positions
    Returns callables  m2px_x(xm, ym)  and  m2px_y(xm, ym).
    """
    A = np.column_stack([mic_xy_m, np.ones(len(mic_xy_m))])
    cx, *_ = lstsq(A, mic_px[:, 0])   # [a, b, tx]
    cy, *_ = lstsq(A, mic_px[:, 1])   # [c, d, ty]

    def m2px(xm, ym):
        xm, ym = np.asarray(xm), np.asarray(ym)
        return (cx[0]*xm + cx[1]*ym + cx[2],
                cy[0]*xm + cy[1]*ym + cy[2])
    return m2px


# ── Load or run calibration ───────────────────────────────────────────────────

if os.path.exists(CALIBRATION_FILE):
    print(f"Loading calibration from {CALIBRATION_FILE}  "
          f"(delete it to recalibrate)")
    mic_px = load_calibration()
else:
    print("No calibration found — starting calibration GUI …")
    mic_px = run_calibration()

m2px = build_transform(mic_xy_m, mic_px)

# ── Load detections ───────────────────────────────────────────────────────────

print("Loading detections …")
detections = pd.read_csv("test_files/acoustic_localization/detections.csv")
base_time  = pytz.timezone("Europe/Oslo").localize(datetime(2024, 1, 1, 12, 0, 0))
detections["start_timestamp"] = [
    base_time + timedelta(seconds=s) for s in detections["start_time"]
]
detections = detections.set_index(["file", "start_time", "end_time", "start_timestamp"])

# ── Run localization ──────────────────────────────────────────────────────────

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

by_species = defaultdict(list)
for e in filtered:
    by_species[e.class_name].append(e)
sorted_species = sorted(by_species.keys())
print(f"  → {len(filtered)} estimates · {len(sorted_species)} species: {sorted_species}")

# ── Plot ──────────────────────────────────────────────────────────────────────

cmap = cm.get_cmap("tab20", max(len(sorted_species), 1))

fig, ax = plt.subplots(figsize=(11, 10))
fig.subplots_adjust(right=0.70)

ax.imshow(image, extent=[0, img_w, img_h, 0], zorder=0)
ax.set_xlim(0, img_w)
ax.set_ylim(img_h, 0)   # image y increases downward

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

# Microphone markers
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
    f"Bird position estimates — {len(filtered)} detections · {len(sorted_species)} species\n"
    f"residual < {MAX_RESIDUAL} m  |  hover for details  |  click legend to toggle",
    fontsize=10,
)
ax.axis("off")

# Legend
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

# Hover tooltip
cursor = mplcursors.cursor(scatter_handles, hover=True)

@cursor.connect("add")
def on_hover(sel):
    sp, xm, ym, rms, ts = point_meta[sel.artist][sel.index]
    sel.annotation.set_text(
        f"{sp}\nx = {xm:.1f} m,  y = {ym:.1f} m\n"
        f"residual = {rms:.2f} m\ntime = {ts}"
    )
    sel.annotation.get_bbox_patch().set(fc="white", alpha=0.92)

plt.show()
save_plot_to_results_folder(fig, "acoustic_localization", "localization_plot.png", dpi=300)
