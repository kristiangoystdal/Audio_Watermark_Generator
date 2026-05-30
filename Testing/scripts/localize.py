#!/usr/bin/env python3
"""
Prepare inputs (BirdNET classification + UTM coords) and localize bird detections.
Run with: /opt/homebrew/bin/python3.11 localize.py
"""

import contextlib
import json
import os
import re
import wave
import pytz
from datetime import datetime, timedelta

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import utm
from birdnetlib import Recording
from birdnetlib.analyzer import Analyzer
from opensoundscape.localization import SynchronizedRecorderArray

# ── Paths ──────────────────────────────────────────────────────────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_RESULTS_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "../results/acoustic_localization"))
os.makedirs(_RESULTS_DIR, exist_ok=True)

# ── Dataset selection ──────────────────────────────────────────────────────────
# Change DATASET to "1" or "2" to switch between recording sessions.

DATASET = "2"

_MIC_COORDS = [(63.418681, 10.407075), (63.418971, 10.406313), (63.419278, 10.406896), (63.418987, 10.407657)]

_DATASETS = {
    "1": {
        "data_dir": os.path.normpath(os.path.join(_SCRIPT_DIR, "../test_files/acoustic_localization/dødens_dal_1")),
        "recording_start": datetime(2000, 1, 1, 0, 0, 0),  # placeholder — no timestamp in filenames
        "mic_filenames": [
            "mic1_cx63.418681_cy10.407074_synced.WAV",
            "mic2_cx63.418972_cy10.406313_synced.WAV",
            "mic3_cx63.419278_cy10.406895_synced.WAV",
            "mic4_cx63.418987_cy10.407657_synced.WAV",
        ],
    },
    "2": {
        "data_dir": os.path.normpath(os.path.join(_SCRIPT_DIR, "../test_files/acoustic_localization/dødens_dal_2")),
        "recording_start": datetime(2026, 5, 30, 12, 18, 16),
        "mic_filenames": [
            "20260530_121816_MIC1_synced.WAV",
            "20260530_121929_MIC2_synced.WAV",
            "20260530_121853_MIC3_synced.WAV",
            "20260530_121810_MIC4_synced.WAV",
        ],
    },
}

_cfg            = _DATASETS[DATASET]
_DATA_DIR       = _cfg["data_dir"]
RECORDING_START = _cfg["recording_start"]
REFERENCE_MIC   = _cfg["mic_filenames"][0]
MIC_GPS         = dict(zip(_cfg["mic_filenames"], _MIC_COORDS))

# ── Tunable parameters ─────────────────────────────────────────────────────────

TIMEZONE = pytz.timezone("Europe/Oslo")

SATELLITE_IMAGE = os.path.join(_DATA_DIR, "SATELLITE_IMAGE.png")
ALIGNMENT_FILE  = os.path.join(_DATA_DIR, "alignment.json")
FORCE_REALIGN   = False  # set to True to redo the image alignment
FORCE_REPROCESS = False  # set to True to re-run BirdNET and regenerate CSVs

AUDIO_DIR = os.path.join(_DATA_DIR, "audio_files")
MIN_CONFIDENCE   = 0.10  # BirdNET confidence threshold
SEGMENT_DURATION = 3.0   # seconds, must match BirdNET window

RESIDUAL_THRESHOLD = 20    # metres — only plot estimates at or below this value
MAX_ARRAY_DIST     = 200   # metres — discard estimates further than this from array centre
CC_THRESHOLD       = 0.01  # minimum cross-correlation score to accept a TDOA
MIN_N_RECEIVERS    = 3     # minimum receivers with detection to attempt localization
MAX_RECEIVER_DIST  = 100   # metres — max distance between receiver pairs for TDOA
CC_FILTER          = "phat"           # cross-correlation filter: "phat", "roth", "scot", or None
LOCALIZATION_ALGORITHM = "least_squares"  # "gillette", "soundfinder", or "least_squares"

# ── Bandpass ranges [low_hz, high_hz] per species ─────────────────────────────
# Ranges are set to cover the core vocalization frequencies for each species.
# Tighten a range to improve TDOA estimation in noisy conditions.

BANDPASS_RANGES = {
    "Abert's Towhee":                 [1000,  5000],
    "Agile Meadow Katydid":           [8000, 16000],
    "Alpine Swift":                   [3000,  8000],
    "American Crow":                  [300,   3000],
    "American Goldfinch":             [2000,  8000],
    "American Pipit":                 [4000, 10000],
    "American Redstart":              [3000,  9000],
    "American Woodcock":              [500,   4000],
    "Andean Gull":                    [500,   4000],
    "Australian Magpie":              [500,   5000],
    "Azure Jay":                      [500,   5000],
    "Bank Swallow":                   [2000,  8000],
    "Bar-tailed Godwit":              [1000,  5000],
    "Barn Swallow":                   [2000,  8000],
    "Bay-backed Shrike":              [1000,  8000],
    "Bird-voiced Treefrog":           [2000,  6000],
    "Black Magpie":                   [500,   5000],
    "Black Redstart":                 [2000, 10000],
    "Black Woodpecker":               [500,   4000],
    "Black-and-gold Cotinga":         [1000,  5000],
    "Black-bellied Plover":           [1000,  5000],
    "Black-billed Cuckoo":            [500,   3000],
    "Black-billed Magpie":            [500,   5000],
    "Black-billed Scythebill":        [1000,  5000],
    "Black-capped Chickadee":         [2000, 10000],
    "Black-chinned Siskin":           [2000,  8000],
    "Black-crowned Night-Heron":      [200,   2000],
    "Black-faced Antbird":            [1000,  5000],
    "Black-headed Saltator":          [1000,  5000],
    "Black-tailed Gnatcatcher":       [3000, 10000],
    "Black-throated Gray Warbler":    [3000,  9000],
    "Black-whiskered Vireo":          [1000,  5000],
    "Blackburnian Warbler":           [4000, 11000],
    "Blackpoll Warbler":              [5000, 12000],
    "Blue-black Grosbeak":            [1000,  5000],
    "Blue-gray Gnatcatcher":          [3000,  9000],
    "Bluethroat":                     [2000,  8000],
    "Bohemian Waxwing":               [4000,  9000],
    "Bonaparte's Gull":               [500,   4000],
    "Boreal Chickadee":               [2000,  8000],
    "Brambling":                      [2000,  8000],
    "Brewer's Blackbird":             [1000,  6000],
    "Brewer's Sparrow":               [3000, 10000],
    "Broad-billed Hummingbird":       [2000,  8000],
    "Brown Honeyeater":               [2000,  8000],
    "Brown Jay":                      [500,   5000],
    "Brown Shrike":                   [1000,  6000],
    "Brown Thornbill":                [2000,  8000],
    "Brown-bellied Swallow":          [2000,  8000],
    "Brown-cheeked Fulvetta":         [2000,  8000],
    "Brown-headed Nuthatch":          [2000,  8000],
    "Bull-headed Shrike":             [1000,  6000],
    "Bullock's Oriole":               [1000,  5000],
    "California Quail":               [500,   4000],
    "Canada Goose":                   [200,   3000],
    "Canyon Wren":                    [2000,  8000],
    "Carolina Chickadee":             [2000, 10000],
    "Carolina Wren":                  [2000,  8000],
    "Carrion Crow":                   [300,   3000],
    "Cassin's Finch":                 [2000,  8000],
    "Cedar Waxwing":                  [4000,  9000],
    "Channel-billed Cuckoo":          [500,   3000],
    "Chiguanco Thrush":               [1000,  5000],
    "Cirl Bunting":                   [2000,  8000],
    "Clay-colored Sparrow":           [3000,  9000],
    "Coal Tit":                       [4000, 10000],
    "Collared Forest-Falcon":         [500,   4000],
    "Common Buzzard":                 [500,   3000],
    "Common Chaffinch":               [2000,  8000],
    "Common Chiffchaff":              [3000,  8000],
    "Common Firecrest":               [5000, 12000],
    "Common Goldeneye":               [500,   3000],
    "Common Grackle":                 [500,   4000],
    "Common Gull":                    [500,   4000],
    "Common House-Martin":            [3000,  8000],
    "Common Loon":                    [300,   3000],
    "Common Nighthawk":               [300,   3000],
    "Common Nightingale":             [2000,  8000],
    "Common Poorwill":                [500,   3000],
    "Common Redpoll":                 [2000,  8000],
    "Common Redstart":                [2000,  8000],
    "Common Rosefinch":               [2000,  8000],
    "Common Scoter":                  [500,   4000],
    "Common Snipe":                   [1000,  6000],
    "Common Swift":                   [4000, 10000],
    "Common True Katydid":            [5000, 12000],
    "Common Virtuoso Katydid":        [5000, 15000],
    "Common Wood-Pigeon":             [300,   2000],
    "Cooper's Hawk":                  [500,   4000],
    "Cuban Ground Cricket":           [4000, 10000],
    "Dark-eyed Junco":                [2000,  8000],
    "Dog":                            [200,   5000],
    "Dunlin":                         [1000,  6000],
    "Dunnock":                        [3000,  8000],
    "Dusky Flycatcher":               [2000,  8000],
    "Dusky Grouse":                   [200,   2000],
    "Eared Poorwill":                 [500,   3000],
    "Eastern Bluebird":               [2000,  8000],
    "Eastern Kingbird":               [2000,  8000],
    "Eastern Narrow-mouthed Toad":    [1000,  5000],
    "Eastern Wood-Pewee":             [2000,  8000],
    "Engine":                         [100,   3000],
    "Eurasian Blackbird":             [2000,  8000],
    "Eurasian Blackcap":              [2000,  8000],
    "Eurasian Blue Tit":              [4000, 10000],
    "Eurasian Coot":                  [500,   4000],
    "Eurasian Dotterel":              [2000,  8000],
    "Eurasian Green Woodpecker":      [1000,  5000],
    "Eurasian Jackdaw":               [500,   4000],
    "Eurasian Jay":                   [500,   5000],
    "Eurasian Magpie":                [500,   5000],
    "Eurasian Moorhen":               [200,   3000],
    "Eurasian Nuthatch":              [1000,  6000],
    "Eurasian Oystercatcher":         [1000,  5000],
    "Eurasian Pygmy-Owl":             [1000,  5000],
    "Eurasian Scops-Owl":             [500,   2000],
    "Eurasian Siskin":                [3000,  9000],
    "Eurasian Sparrowhawk":           [500,   4000],
    "Eurasian Thick-knee":            [500,   5000],
    "Eurasian Three-toed Woodpecker": [1000,  5000],
    "Eurasian Treecreeper":           [4000, 10000],
    "Eurasian Woodcock":              [300,   3000],
    "Eurasian Wren":                  [2000, 10000],
    "European Goldfinch":             [2000,  8000],
    "European Greenfinch":            [2000,  8000],
    "European Pied Flycatcher":       [2000,  8000],
    "European Robin":                 [2000,  8000],
    "European Serin":                 [3000, 10000],
    "European Stonechat":             [3000,  8000],
    "Evening Grosbeak":               [2000,  6000],
    "False Robust Conehead":          [5000, 15000],
    "Field Sparrow":                  [3000,  8000],
    "Fieldfare":                      [1000,  5000],
    "Fish Crow":                      [300,   3000],
    "Goldcrest":                      [5000, 12000],
    "Golden-crowned Kinglet":         [5000, 12000],
    "Grace's Warbler":                [3000,  8000],
    "Gray Heron":                     [200,   2000],
    "Gray Kingbird":                  [1000,  5000],
    "Gray Vireo":                     [2000,  6000],
    "Gray Wagtail":                   [3000,  8000],
    "Gray Wolf":                      [200,   3000],
    "Gray-throated Leaftosser":       [1000,  5000],
    "Great Gray Shrike":              [1000,  6000],
    "Great Horned Owl":               [200,   1500],
    "Great Kiskadee":                 [1000,  6000],
    "Great Spotted Woodpecker":       [1000,  5000],
    "Great Tit":                      [3000,  8000],
    "Greater Prairie-Chicken":        [200,   2000],
    "Greater Yellowlegs":             [1000,  5000],
    "Green-backed Tit":               [3000,  9000],
    "Green-breasted Mountain-gem":    [3000,  8000],
    "Green-rumped Parrotlet":         [2000,  8000],
    "Green-tailed Towhee":            [2000,  6000],
    "Greenhouse Frog":                [1000,  5000],
    "Hammond's Flycatcher":           [2000,  8000],
    "Handsome Meadow Katydid":        [5000, 15000],
    "Hawaii Amakihi":                 [2000,  8000],
    "Hellmayr's Pipit":               [3000,  8000],
    "Hermit Warbler":                 [4000, 10000],
    "Herring Gull":                   [500,   4000],
    "Hooded Crow":                    [300,   3000],
    "Hooded Oriole":                  [1000,  5000],
    "Hooded Siskin":                  [2000,  8000],
    "House Finch":                    [2000,  8000],
    "House Sparrow":                  [2000,  8000],
    "Human non-vocal":                [100,   4000],
    "Human vocal":                    [100,   4000],
    "Hutton's Vireo":                 [2000,  6000],
    "Indian Paradise-Flycatcher":     [2000,  8000],
    "Indian Scops-Owl":               [500,   3000],
    "Indian White-eye":               [3000,  9000],
    "Jungle Babbler":                 [500,   5000],
    "Kelp Gull":                      [500,   4000],
    "Lady Amherst's Pheasant":        [500,   3000],
    "Large-billed Tern":              [1000,  5000],
    "Large-headed Flatbill":          [2000,  8000],
    "Lesser Black-backed Gull":       [500,   4000],
    "Lesser Goldfinch":               [2000,  8000],
    "Lesser Kestrel":                 [500,   4000],
    "Lesser Spotted Woodpecker":      [1000,  5000],
    "Limpkin":                        [500,   3000],
    "Lineated Woodpecker":            [500,   4000],
    "Lined Seedeater":                [2000,  8000],
    "Little Bittern":                 [200,   2000],
    "Little Owl":                     [500,   3000],
    "Little Raven":                   [300,   3000],
    "Little Ringed Plover":           [1000,  5000],
    "Long-eared Owl":                 [200,   2000],
    "Long-tailed Jaeger":             [500,   4000],
    "MacGillivray's Warbler":         [2000,  8000],
    "Mangrove Vireo":                 [1000,  5000],
    "Marbled Murrelet":               [1000,  5000],
    "Marsh Tit":                      [3000,  8000],
    "Masked Shrike":                  [1000,  6000],
    "Micronesian Myzomela":           [2000,  8000],
    "Middle Spotted Woodpecker":      [1000,  5000],
    "Mistle Thrush":                  [2000,  8000],
    "Mountain Cacique":               [1000,  5000],
    "Mountain Chickadee":             [2000,  8000],
    "Mourning Dove":                  [300,   2000],
    "Mute Swan":                      [200,   2000],
    "Narcissus Flycatcher":           [2000,  8000],
    "New Zealand Fantail":            [2000,  8000],
    "Noisy Friarbird":                [500,   5000],
    "Northern Goshawk":               [500,   4000],
    "Northern Lapwing":               [500,   5000],
    "Northern Pygmy-Owl":             [1000,  5000],
    "Northern Rough-winged Swallow":  [2000,  8000],
    "Oak Titmouse":                   [2000,  8000],
    "Olive Warbler":                  [3000,  9000],
    "Olive-winged Bulbul":            [1000,  5000],
    "Oriental Greenfinch":            [2000,  8000],
    "Ortolan Bunting":                [2000,  8000],
    "Osprey":                         [500,   4000],
    "Pacific Screech-Owl":            [500,   3000],
    "Painted Bunting":                [2000,  8000],
    "Pale-breasted Thrush":           [1000,  5000],
    "Pallas's Leaf Warbler":          [5000, 12000],
    "Paradise Shelduck":              [500,   3000],
    "Peregrine Falcon":               [500,   4000],
    "Phainopepla":                    [2000,  8000],
    "Pine Grosbeak":                  [2000,  6000],
    "Pine Siskin":                    [2000,  8000],
    "Pine Warbler":                   [2000,  6000],
    "Plain-colored Seedeater":        [2000,  8000],
    "Power tools":                    [100,   5000],
    "Red Kite":                       [500,   3000],
    "Red-bellied Macaw":              [1000,  5000],
    "Red-breasted Flycatcher":        [2000,  8000],
    "Red-eyed Vireo":                 [1000,  5000],
    "Red-shouldered Hawk":            [500,   3000],
    "Red-winged Blackbird":           [500,   4000],
    "Redwing":                        [2000,  7000],
    "Reed Bunting":                   [2000,  8000],
    "Ring Ouzel":                     [2000,  7000],
    "Ring-billed Gull":               [500,   4000],
    "Ring-necked Duck":               [500,   3000],
    "Ring-necked Pheasant":           [200,   3000],
    "Rock Bunting":                   [2000,  8000],
    "Rock Ptarmigan":                 [500,   3000],
    "Rock Wren":                      [2000,  8000],
    "Rook":                           [300,   3000],
    "Ross's Gull":                    [500,   4000],
    "Round-tipped Conehead":          [5000, 15000],
    "Ruddy Shelduck":                 [300,   3000],
    "Ruddy Turnstone":                [1000,  5000],
    "Rufous-capped Warbler":          [2000,  8000],
    "Rufous-naped Wren":              [1000,  5000],
    "Rufous-tailed Robin":            [2000,  8000],
    "Russet-crowned Warbler":         [3000,  8000],
    "Rusty-margined Flycatcher":      [1000,  5000],
    "Sagebrush Sparrow":              [2000,  8000],
    "Sardinian Warbler":              [2000,  8000],
    "Savi's Warbler":                 [2000,  8000],
    "Scarlet Tanager":                [1000,  5000],
    "Seaside Sparrow":                [2000,  8000],
    "Short-billed Gull":              [500,   4000],
    "Short-winged Meadow Katydid":    [5000, 15000],
    "Siren":                          [100,   5000],
    "Slender-billed Parakeet":        [1000,  5000],
    "Snow Goose":                     [300,   3000],
    "Song Thrush":                    [2000,  8000],
    "Southern Beardless-Tyrannulet":  [3000,  8000],
    "Southern Lapwing":               [500,   4000],
    "Spangled Drongo":                [500,   5000],
    "Spotted Crake":                  [500,   4000],
    "Spotted Flycatcher":             [2000,  8000],
    "Stock Dove":                     [200,   1500],
    "Stygian Owl":                    [200,   2000],
    "Syrian Woodpecker":              [1000,  5000],
    "Tawny Owl":                      [300,   2000],
    "Tit-like Dacnis":                [2000,  8000],
    "Townsend's Warbler":             [4000, 10000],
    "Tree Pipit":                     [2000,  8000],
    "Tufted Titmouse":                [2000,  8000],
    "Tundra Swan":                    [200,   2000],
    "Varied Thrush":                  [1000,  5000],
    "Vesper Sparrow":                 [2000,  6000],
    "Vinous-throated Parrotbill":     [2000,  8000],
    "Warbling Vireo":                 [2000,  6000],
    "Water Rail":                     [500,   5000],
    "Wattled Lapwing":                [500,   4000],
    "Wedge-tailed Shearwater":        [500,   3000],
    "Welcome Swallow":                [2000,  8000],
    "Western Wood-Pewee":             [2000,  8000],
    "White Wagtail":                  [2000,  8000],
    "White-bellied Woodpecker":       [500,   4000],
    "White-breasted Nuthatch":        [1000,  5000],
    "White-crowned Manakin":          [2000,  8000],
    "White-tipped Dove":              [300,   2000],
    "White-winged Dove":              [300,   2000],
    "Whooper Swan":                   [200,   2000],
    "Wild Turkey":                    [200,   3000],
    "Willet":                         [1000,  5000],
    "Willie-wagtail":                 [2000,  8000],
    "Wilson's Phalarope":             [500,   3000],
    "Wood Duck":                      [500,   3000],
    "Wood Lark":                      [2000,  6000],
    "Wood Sandpiper":                 [1000,  5000],
    "Wood Thrush":                    [1000,  5000],
    "Wood Warbler":                   [3000,  9000],
    "Wren-like Rushbird":             [1000,  5000],
    "Wrentit":                        [2000,  8000],
    "Yellow-bellied Sapsucker":       [500,   3000],
    "Yellow-hooded Blackbird":        [1000,  5000],
    "Yellow-legged Gull":             [500,   4000],
    "Yellow-rumped Warbler":          [2000,  8000],
    "Yellow-throated Bunting":        [2000,  8000],
    "Yellow-throated Toucan":         [500,   4000],
    "Yellowhammer":                   [2000,  6000],
}

# ── Generate aru_coords.csv and detections.csv (if needed) ────────────────────

def _get_wav_duration(path):
    with contextlib.closing(wave.open(path, "r")) as f:
        return f.getnframes() / float(f.getframerate())

_ARU_COORDS_CSV = os.path.join(_DATA_DIR, "aru_coords.csv")
_DETECTIONS_CSV = os.path.join(_DATA_DIR, "detections.csv")

if FORCE_REPROCESS or not (os.path.exists(_ARU_COORDS_CSV) and os.path.exists(_DETECTIONS_CSV)):
    print("Generating aru_coords.csv and detections.csv...")

    utm_coords = {f: utm.from_latlon(lat, lon)[:2] for f, (lat, lon) in MIC_GPS.items()}
    ref_e, ref_n = utm_coords[REFERENCE_MIC]
    rel_coords = {f: (e - ref_e, n - ref_n) for f, (e, n) in utm_coords.items()}

    coords_df = pd.DataFrame(
        {"x": {f: x for f, (x, _) in rel_coords.items()},
         "y": {f: y for f, (_, y) in rel_coords.items()}}
    )
    coords_df.index.name = ""
    coords_df.to_csv(_ARU_COORDS_CSV)

    analyzer = Analyzer()
    raw_detections = {}
    for filename in MIC_GPS:
        path = os.path.join(AUDIO_DIR, filename)
        recording = Recording(analyzer, path, min_conf=MIN_CONFIDENCE)
        recording.analyze()
        raw_detections[filename] = recording.detections
        print(f"  {filename}: {len(recording.detections)} detections above {MIN_CONFIDENCE:.0%}")

    all_species = sorted({d["common_name"] for dets in raw_detections.values() for d in dets})

    rows = []
    for filename in MIC_GPS:
        path = os.path.join(AUDIO_DIR, filename)
        duration = _get_wav_duration(path)
        det_map: dict = {}
        for d in raw_detections[filename]:
            key = (float(d["start_time"]), float(d["end_time"]))
            det_map.setdefault(key, set()).add(d["common_name"])
        start = 0.0
        while start + SEGMENT_DURATION <= duration:
            end = start + SEGMENT_DURATION
            detected = det_map.get((start, end), set())
            row = {"file": filename, "start_time": start, "end_time": end}
            for sp in all_species:
                row[sp] = 1.0 if sp in detected else 0.0
            rows.append(row)
            start = end

    pd.DataFrame(rows).to_csv(_DETECTIONS_CSV, index=False)
    print(f"Written detections.csv ({len(rows)} rows, {len(all_species)} species)")

# ── Load inputs ────────────────────────────────────────────────────────────────

aru_coords = pd.read_csv(_ARU_COORDS_CSV, index_col=0)
aru_coords.index = [f"{AUDIO_DIR}/{f}" for f in aru_coords.index]

detections = pd.read_csv(_DETECTIONS_CSV)
detections["file"] = detections["file"].apply(lambda f: f"{AUDIO_DIR}/{f}")

base_ts = TIMEZONE.localize(RECORDING_START)
detections["start_timestamp"] = [
    base_ts + timedelta(seconds=s) for s in detections["start_time"]
]
detections = detections.set_index(["file", "start_time", "end_time", "start_timestamp"])

array = SynchronizedRecorderArray(aru_coords)

# ── Localize ───────────────────────────────────────────────────────────────────

print("Running localization...")
position_estimates = array.localize_detections(
    detections,
    max_receiver_dist=MAX_RECEIVER_DIST,
    min_n_receivers=MIN_N_RECEIVERS,
    localization_algorithm=LOCALIZATION_ALGORITHM,
    cc_threshold=CC_THRESHOLD,
    cc_filter=CC_FILTER,
    bandpass_ranges=BANDPASS_RANGES,
    residual_threshold=RESIDUAL_THRESHOLD,
)

array_centre = (aru_coords["x"].mean(), aru_coords["y"].mean())
position_estimates = [
    e for e in position_estimates
    if np.hypot(e.location_estimate[0] - array_centre[0],
                e.location_estimate[1] - array_centre[1]) <= MAX_ARRAY_DIST
]

print(f"Total position estimates (residual ≤ {RESIDUAL_THRESHOLD} m, within {MAX_ARRAY_DIST} m of array centre): {len(position_estimates)}")

# ── Satellite image alignment ─────────────────────────────────────────────────

img = plt.imread(SATELLITE_IMAGE)

if not FORCE_REALIGN and os.path.exists(ALIGNMENT_FILE):
    with open(ALIGNMENT_FILE) as f:
        raw = json.load(f)
    pixel_positions = {os.path.basename(k): np.array(v) for k, v in raw.items()}
    print(f"Loaded alignment from {ALIGNMENT_FILE}")
else:
    print("Starting interactive alignment — click each ARU location in the image.")
    fig_align, ax_align = plt.subplots(figsize=(14, 10))
    ax_align.imshow(img)
    ax_align.axis("off")
    plt.tight_layout()
    plt.show(block=False)

    pixel_positions = {}
    for aru_name in aru_coords.index:
        fname = os.path.basename(aru_name)
        m = re.search(r"[Mm][Ii][Cc](\d+)", fname)
        short = f"MIC {m.group(1)}" if m else fname
        ax_align.set_title(f"Click on the location of {short}", fontsize=13)
        plt.draw()
        pts = plt.ginput(1, timeout=0)
        px, py = pts[0]
        pixel_positions[os.path.basename(aru_name)] = [px, py]
        ax_align.plot(px, py, "r^", markersize=12, zorder=5)
        ax_align.annotate(short, (px, py), textcoords="offset points",
                          xytext=(6, 6), color="red", fontsize=10, fontweight="bold")
        plt.draw()

    plt.close(fig_align)
    with open(ALIGNMENT_FILE, "w") as f:
        json.dump(pixel_positions, f, indent=2)
    print(f"Alignment saved to {ALIGNMENT_FILE}")

# Fit affine transform UTM → pixel using ARU positions as control points
utm_pts = aru_coords[["x", "y"]].values
pix_pts = np.array([pixel_positions[os.path.basename(n)] for n in aru_coords.index])
A_ctrl  = np.column_stack([utm_pts, np.ones(len(utm_pts))])
M, _, _, _ = np.linalg.lstsq(A_ctrl, pix_pts, rcond=None)  # M shape (3, 2)

def utm_to_pixel(xs, ys):
    pts = np.column_stack([xs, ys, np.ones(len(xs))])
    return pts @ M  # returns (N, 2) array of [px, py]

# ── Plot ───────────────────────────────────────────────────────────────────────

species_list = sorted({e.class_name for e in position_estimates})
colors = cm.tab20(np.linspace(0, 1, max(len(species_list), 1)))
color_map = {sp: colors[i] for i, sp in enumerate(species_list)}

plt.rcParams.update({
    "font.size": 9,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
})

fig, ax = plt.subplots(figsize=(6.5, 4.5))
ax.imshow(img, zorder=0)
ax.axis("off")

# ARU positions — use clicked pixel positions directly
aru_pix = np.array([pixel_positions[os.path.basename(n)] for n in aru_coords.index])
ax.scatter(aru_pix[:, 0], aru_pix[:, 1],
           marker="^", color="red", s=60, zorder=5, label="ARU")
for name, (px, py) in zip(aru_coords.index, aru_pix):
    fname = os.path.basename(name)
    m = re.search(r"[Mm][Ii][Cc](\d+)", fname)
    short = f"MIC {m.group(1)}" if m else fname
    ax.annotate(short, (px, py), textcoords="offset points", xytext=(4, 4),
                fontsize=7, color="white", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="black", alpha=0.6))

# Detections per species
for sp in species_list:
    estimates = [e for e in position_estimates if e.class_name == sp]
    xs = [e.location_estimate[0] for e in estimates]
    ys = [e.location_estimate[1] for e in estimates]
    pix = utm_to_pixel(xs, ys)
    ax.scatter(pix[:, 0], pix[:, 1],
               color=color_map[sp], label=f"{sp} ({len(estimates)})", alpha=1.0, edgecolors="black", linewidths=0.6, s=50, zorder=4)

if species_list:
    ax.legend(
        bbox_to_anchor=(1.01, 1), loc="upper left",
        fontsize=8, markerscale=1.0, framealpha=0.9,
        borderpad=0.5, handletextpad=0.4,
    )

plt.tight_layout()
plt.savefig(os.path.join(_RESULTS_DIR, "localizations.png"), dpi=300, bbox_inches="tight")
print(f"Saved localizations.png to {_RESULTS_DIR}")
plt.show()
