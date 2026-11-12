#!/usr/bin/env python3
"""
DUV Autocollimator Control GUI  (software revision B2.1)
=========================================================

Control and measurement software for a camera-based DUV autocollimator.

Hardware
--------
- Sensor: Basler ace2 a2A4504-18umPRO (Sony IMX541, 4504 x 4504 px,
  2.74 um pixels, global shutter, 18 fps max, USB3 Vision, C-mount).
- DUV source (240-270 nm), coaxial LED illumination, and focus are all
  set MANUALLY on the instrument. The software controls the camera only.

What the software does
----------------------
- Live acquisition from the camera on a background thread.
- Sub-pixel centroid of the return spot at full sensor resolution.
- Calibrated angle readout (AZ / EL) relative to a "Set Calibrated Zero"
  reference, in arc-seconds or micro-radians:

      arc-sec per pixel = pixel_pitch / (2 * collimator_focal_length) * 206265

  (factor of 2: a mirror tilt of theta deflects the return by 2*theta).
- Metrology statistics: RMS noise, 2-sigma repeatability, signal level,
  saturation detection, spot size (focus aid), sensor temperature.
- Tolerance alert (green/red readout + soft chime on change).
- Capture: window screenshot, short screen recording, raw full-resolution
  frame (16-bit TIFF), and CSV data logging.
- Saved system profiles (persist across restarts), including each
  system's calibrated zero, tied to the camera serial number; export /
  import to move them between computers.
- The whole session (every setting, mode, source, calibration, window
  size, last profile) is saved when the app closes and every 30 s, and
  restored at the next launch. Saved in ~/.duv_autocollimator/, outside
  the app, so it survives installing a new revision.
- Display themes, including palettes readable through laser safety glasses.

Modes (display only; both use the same live camera)
---------------------------------------------------
- Alignment: Source/Return points on a graph of the whole sensor.
- Metrology: the live camera image (zoomed to the ROI) with overlays.

Camera sources
--------------
- Camera:    the real Basler over USB3 (requires pypylon).
- Emulator:  pylon's built-in camera emulator (no hardware).
- Simulator: synthetic spot generator; enables Test Point mode to check
             the centroid -> angle math against a value you choose.

Setup for the real camera
-------------------------
1. Install the Basler pylon Software Suite (USB driver + pylon Viewer).
2. Install pypylon INTO THE SAME PYTHON that runs this script:
       python3 -m pip install pypylon
3. Close pylon Viewer before connecting (only one program can open the
   camera at a time).
4. Plug the camera into a USB 3 port (blue / "SS"). On USB 2 it will
   enumerate but cannot stream full frames.
5. Before opening the GUI, run the bring-up check from a terminal:
       python3 duv_autocollimator_gui.py --check-camera
   (or  --check-camera --emulator  to rehearse without hardware).
   It prints every camera feature the GUI relies on and grabs frames.
6. If several Basler cameras are attached, pick one by serial number:
       DUV_CAMERA_SERIAL=40123456 python3 duv_autocollimator_gui.py

Sensor coordinates
------------------
All pixel coordinates (centroid, calibrated zero, plot axes) are
continuous sensor coordinates: pixel i covers [i, i+1), so a pixel's
center is at i + 0.5 and the exact optical center of the sensor is
(width / 2, height / 2), e.g. (2252, 2252) on the a2A4504.

Changes in B2.1
---------------
- Fixed layout: the three columns are a fixed share of the window, so
  no text, mode, or source change can resize any panel.
- Plots use fixed-pixel square axes: the left graph keeps exactly the
  same size and position in Alignment and Metrology, and with Camera,
  Emulator or Simulator.
- Grid lines are laid out symmetrically about the sensor center, so the
  Source crosshair sits on the center grid line.
- Centroid/display half-pixel offset removed (see Sensor coordinates).
- Camera bring-up: pylon emulator is enabled at startup (no restart
  needed), serial-number selection, USB-link check, sensor temperature
  selector, pixel-format fallback, emulator fed with a synthetic DUV
  return spot so the full pylon -> centroid path can be tested.
"""

# The pylon emulator must be enabled BEFORE pylon first lists devices.
# Setting it here (before pypylon is ever imported) means the Emulator
# source works without restarting the app; the real "Camera" source
# filters emulated devices out.
import os as _os
_os.environ.setdefault("PYLON_CAMEMU", "1")

import csv
import glob
import json
import math
import os
import platform
import random
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import wave
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, colorchooser

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from matplotlib.ticker import FixedLocator, FuncFormatter  # noqa: E402

try:
    from PIL import Image, ImageDraw, ImageGrab
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

SOFTWARE_REV = "B2.4"

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------

ARC_SEC_PER_RAD = 206264.806
MICRO_RAD_PER_ARC_SEC = 1_000_000.0 / ARC_SEC_PER_RAD  # ~4.84814

SAMPLE_RATES = [1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0]
# "Millimeter" = linear displacement of the Return spot from the Source
# ON THE SENSOR (pixels x pixel pitch). Used to verify the centroid with a
# known linear move (translation stage) before trusting angular readings.
DATA_UNITS = ["Arc-Sec", "Micro-Radian", "Millimeter"]
MODES = ["Alignment", "Metrology"]
SOURCES = ["Camera", "Emulator", "Simulator"]
DEFAULT_MODE = "Alignment"
DEFAULT_SOURCE = "Camera"
STATUS_LINES = 3                 # camera status box is always exactly this many lines tall

FIXED_FILTER = "Broadband (240-270 nm)"

# ---- Basler ace2 a2A4504-18umPRO (Edmund #16-635) ----
CAMERA_MODEL = "a2A4504-18umPRO"
CAMERA_DISPLAY_NAME = "Basler ace2 a2A4504-18umPRO"
CAMERA_SENSOR = "Sony IMX541"
CAMERA_WIDTH_PX = 4504
CAMERA_HEIGHT_PX = 4504
CAMERA_PIXEL_UM = 2.74
CAMERA_MAX_FPS = 18.0
COLLIMATOR_FOCAL_MM_DEFAULT = 200.0   # <-- set to your collimator objective's focal length
PIXEL_FORMATS = [("Mono8", "8-bit"), ("Mono12", "12-bit")]
# Unpacked formats only (GetArray can't unpack "...p" packed formats);
# first one the camera offers wins.
PIXEL_FORMAT_FALLBACKS = {"Mono8": ["Mono8"], "Mono12": ["Mono12", "Mono10", "Mono16", "Mono8"]}
CAMERA_SERIAL = os.environ.get("DUV_CAMERA_SERIAL", "").strip()   # "" = first a2A4504 found
USB3_MIN_LINK_BPS = 400e6        # below this the camera is on a USB 2 port
# Approximate USB3 payload limit for the ace2 (~360 MB/s): full sensor
# is ~20.3 MB/frame in Mono8 (~17 fps) and ~40.6 MB/frame in Mono12 (~9 fps)
USB3_PAYLOAD_BPS = 360e6
ROI_CHOICES = ["Full sensor", "2048 × 2048", "1024 × 1024", "512 × 512", "256 × 256"]

# Camera setting defaults / UI ranges (real limits are read from the
# camera on connect and the controls are re-ranged to match)
EXPOSURE_DEFAULT_MS = 10.0
EXPOSURE_MIN_MS = 0.02           # 20 us
EXPOSURE_MAX_MS = 1000.0         # 1 s
GAIN_MAX_DB = 24.0
BLACK_LEVEL_MAX = 255.0
FRAME_RATE_DEFAULT = 10.0

DISPLAY_MAX_PX = 500             # live image is block-averaged to ~this size for DISPLAY only
CHART_HISTORY = 300
STATS_WINDOW = 50                # samples used for RMS noise / repeatability
TEMP_POLL_S = 2.0                # how often to read sensor temperature

# ---- Simulator (no hardware) ----
FRAME_SIZE = 380                 # simulated sensor, pixels (square)
ARCSEC_PER_PIXEL_SIM = 6.0       # simulator's fixed plate scale
SIM_SPOT_PEAK = 190.0            # counts at the reference exposure and 0 dB gain
SIM_BACKGROUND = 16.0
SIM_REF_EXPOSURE_MS = 10.0

# ---- Emulator spot images (pylon emulator plays these back as frames) ----
EMU_SPOT_FRAMES = 8
EMU_SPOT_SIGMA_PX = 4.0
EMU_SPOT_OFFSET = (0.06, -0.04)  # spot offset from sensor center, fraction of sensor size

# ---- Plot geometry (pixels) — fixed, so plots never change size ----
FEED_MARGINS = (58, 12, 10, 40)  # left, right, top, bottom
XY_MARGINS = (58, 12, 10, 40)
INFO_LINES = 2                   # bottom info strip is always this many lines tall

VIDEO_RECORD_SECONDS = 5.0
PROFILE_DIR = os.path.join(os.path.expanduser("~"), ".duv_autocollimator")
PROFILE_FILE = os.path.join(PROFILE_DIR, "profiles.json")
SESSION_FILE = os.path.join(PROFILE_DIR, "session.json")   # everything restored at next launch
AUTOSAVE_S = 30.0                # also save the session this often (in case of a crash)

# Fixed literal colors for the AZ/EL tolerance readout — intentionally
# NOT theme-dependent, so it always reads green = good / red = bad.
TOLERANCE_OK_COLOR = "#2ecc71"
TOLERANCE_BAD_COLOR = "#e74c3c"
NO_SIGNAL_COLOR = "#7f8c8d"

DEFAULT_THEME = "Dark (default)"

# ----------------------------------------------------------------------
# Display themes
# ----------------------------------------------------------------------
# "ok"/"bad" drive status indicators. The two "Laser Glasses" themes
# avoid blue (blocked by blue-blocking OD glasses) and red/green pairs,
# and push contrast up so readouts stay legible through a tint.

THEMES = {
    "Dark (default)": {
        "root_bg":       "#d9d9d9",
        "panel_bg":      "#d9d9d9",
        "fg":            "#000000",
        "entry_bg":      "#ffffff",
        "entry_fg":      "#000000",
        "readout_bg":    "#ffffff",
        "readout_fg":    "#000000",
        "border":        "#808080",
        "plot_bg":       "#000000",
        "plot_fg":       "#ffffff",
        "grid":          "#4d4d00",
        "marker":        "#ffffff",
        "ok":            "#39d353",
        "bad":           "#e74c3c",
        "neutral":       "#808080",
        "exit_bg":       "#c0392b",
        "exit_fg":       "#ffffff",
    },
    "Light": {
        "root_bg":       "#f2f2f2",
        "panel_bg":      "#f2f2f2",
        "fg":            "#000000",
        "entry_bg":      "#ffffff",
        "entry_fg":      "#000000",
        "readout_bg":    "#ffffff",
        "readout_fg":    "#000000",
        "border":        "#a6a6a6",
        "plot_bg":       "#ffffff",
        "plot_fg":       "#000000",
        "grid":          "#c9c9c9",
        "marker":        "#000000",
        "ok":            "#1b7a3d",
        "bad":           "#b3261e",
        "neutral":       "#a6a6a6",
        "exit_bg":       "#b3261e",
        "exit_fg":       "#ffffff",
    },
    "Night / Low-Glare": {
        "root_bg":       "#141414",
        "panel_bg":      "#1c1c1c",
        "fg":            "#e6e6e6",
        "entry_bg":      "#2a2a2a",
        "entry_fg":      "#e6e6e6",
        "readout_bg":    "#000000",
        "readout_fg":    "#ff8c1a",
        "border":        "#3a3a3a",
        "plot_bg":       "#000000",
        "plot_fg":       "#e6e6e6",
        "grid":          "#333300",
        "marker":        "#ff8c1a",
        "ok":            "#39d353",
        "bad":           "#e74c3c",
        "neutral":       "#555555",
        "exit_bg":       "#7a1f1f",
        "exit_fg":       "#ffffff",
    },
    "Laser Glasses \u2013 Blue-Blocking (Dark)": {
        # Tuned for blue-blocking OD glasses: the tint filters out the
        # blue channel almost entirely, so blue (and anything mixed
        # from it, like purple) reads as dim gray or black through the
        # lenses. Every accent color here is built from red/amber/
        # yellow instead, which pass through cleanly, and status colors
        # use red vs. amber/yellow (not red/green) so the two stay
        # distinguishable from each other under the tint.
        "root_bg":       "#0d0d0d",
        "panel_bg":      "#141414",
        "fg":            "#f5f5f5",
        "entry_bg":      "#1f1f1f",
        "entry_fg":      "#f5f5f5",
        "readout_bg":    "#000000",
        "readout_fg":    "#ffffff",
        "border":        "#3d3d3d",
        "plot_bg":       "#000000",
        "plot_fg":       "#ffffff",
        "grid":          "#3a2020",
        "marker":        "#ff3333",
        "ok":            "#ff3333",
        "bad":           "#ffcc00",
        "neutral":       "#666666",
        "exit_bg":       "#661111",
        "exit_fg":       "#ffffff",
    },
    "Laser Glasses \u2013 Blue-Blocking (Light)": {
        # Same blue-free palette as the dark version, on a light
        # background for bright-room / daytime bench work.
        "root_bg":       "#f2f2f2",
        "panel_bg":      "#f2f2f2",
        "fg":            "#000000",
        "entry_bg":      "#ffffff",
        "entry_fg":      "#000000",
        "readout_bg":    "#ffffff",
        "readout_fg":    "#000000",
        "border":        "#a6a6a6",
        "plot_bg":       "#ffffff",
        "plot_fg":       "#000000",
        "grid":          "#e6d6d6",
        "marker":        "#b3261e",
        "ok":            "#b3261e",
        "bad":           "#a3790a",
        "neutral":       "#a6a6a6",
        "exit_bg":       "#8c1c1c",
        "exit_fg":       "#ffffff",
    },
}


# ----------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------

def pick_font_family(root, candidates, fallback):
    """Return the first installed font family from candidates. Fonts
    like "Consolas" don't exist on macOS; Tk silently substitutes a
    wider proportional font, which made the AZ/EL readout overflow."""
    try:
        installed = {f.lower() for f in tkfont.families(root)}
    except tk.TclError:
        return fallback
    for name in candidates:
        if name.lower() in installed:
            return name
    return fallback


def list_serial_ports():
    """Serial ports that actually exist on this computer (for the
    reserved instrument link). No pyserial dependency."""
    system = platform.system()
    if system == "Windows":
        return [f"COM{i}" for i in range(1, 17)]
    if system == "Darwin":
        ports = sorted(glob.glob("/dev/cu.*"))
    else:
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports or ["(none found)"]


def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip("#")
    return tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))


def contrast_text(hex_color):
    r, g, b = hex_to_rgb(hex_color)
    luminance = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    return "#000000" if luminance > 0.55 else "#ffffff"


def centered_ticks(lo, hi, center, target=8):
    """Tick positions spaced symmetrically about `center`, so a grid line
    always runs through the center (where the Source crosshair sits) and
    the grid looks the same on both sides of it."""
    span = float(hi) - float(lo)
    if span <= 0:
        return [center]
    raw = span / target
    mag = 10 ** math.floor(math.log10(raw))
    step = next((n * mag for n in (1, 2, 2.5, 5, 10) if n * mag >= 0.8 * raw), 10 * mag)
    k_lo = math.ceil((lo - center) / step - 1e-9)
    k_hi = math.floor((hi - center) / step + 1e-9)
    return [center + k * step for k in range(k_lo, k_hi + 1)]


def tick_label(value, _pos=None):
    """Whole numbers without decimals, everything else with one."""
    if abs(value - round(value)) < 1e-6:
        return f"{value:.0f}"
    return f"{value:.1f}"


def pin_square_axes(fig, ax, margins):
    """Keep `ax` a square of fixed pixel margins inside its canvas,
    centered, whatever the canvas size. Tick-label width, axis labels,
    mode or data can then never move or resize the plot (matplotlib's
    automatic layout shifts the axes whenever tick labels change width,
    e.g. "350" on the simulator vs "4000" on the camera)."""
    left, right, top, bottom = margins

    def _layout(_event=None):
        w_px, h_px = fig.get_size_inches() * fig.dpi
        if w_px < 10 or h_px < 10:
            return
        avail_w, avail_h = w_px - left - right, h_px - top - bottom
        side = max(20.0, min(avail_w, avail_h))
        x0 = left + max(0.0, (avail_w - side) / 2.0)
        y0 = bottom + max(0.0, (avail_h - side) / 2.0)
        ax.set_position([x0 / w_px, y0 / h_px, side / w_px, side / h_px])
        # axis titles at a fixed pixel distance from the plot frame
        ax.yaxis.set_label_coords(-(left - 14) / side, 0.5)
        ax.xaxis.set_label_coords(0.5, -(bottom - 14) / side)

    fig.canvas.mpl_connect("resize_event", _layout)
    _layout()
    return _layout


def ensure_emulator_spot_images(width, height, max_value_8bit=235):
    """Write a short sequence of 8-bit PNG frames containing a Gaussian
    return spot that wanders slightly, for the pylon emulator to play
    back (ImageFileMode). This exercises the REAL pylon grab path —
    pixel format, ROI offsets, flips — with a spot whose true position
    is known. Returns the folder path, or None if it can't be written."""
    if not PIL_AVAILABLE:
        return None
    folder = os.path.join(PROFILE_DIR, f"emulator_spot_{width}x{height}")
    try:
        if len(glob.glob(os.path.join(folder, "*.png"))) == EMU_SPOT_FRAMES:
            return folder
        os.makedirs(folder, exist_ok=True)
        cx0 = width * (0.5 + EMU_SPOT_OFFSET[0])
        cy0 = height * (0.5 + EMU_SPOT_OFFSET[1])
        r = int(6 * EMU_SPOT_SIGMA_PX) + 4
        rng = np.random.default_rng(7)
        for i in range(EMU_SPOT_FRAMES):
            a = 2.0 * math.pi * i / EMU_SPOT_FRAMES
            cx, cy = cx0 + 1.5 * math.cos(a), cy0 + 1.0 * math.sin(a)
            img = np.full((height, width), 12, dtype=np.uint8)
            x0, y0 = int(cx) - r, int(cy) - r
            yy, xx = np.mgrid[y0:y0 + 2 * r, x0:x0 + 2 * r]
            # continuous coordinates: pixel i's center is at i + 0.5
            spot = (max_value_8bit - 12) * np.exp(-((xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2)
                                                  / (2 * EMU_SPOT_SIGMA_PX ** 2))
            spot += rng.normal(0, 1.5, spot.shape)
            img[y0:y0 + 2 * r, x0:x0 + 2 * r] = np.clip(12 + spot, 0, 255).astype(np.uint8)
            Image.fromarray(img).save(os.path.join(folder, f"spot_{i:02d}.png"), compress_level=1)
        return folder
    except Exception as exc:
        print(f"[emulator] could not write spot images: {exc}")
        return None


def write_json_atomic(path, data):
    """Write JSON so a crash mid-write can never leave a corrupt file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def roi_size_from_choice(choice):
    """'1024 × 1024' -> 1024; 'Full sensor' -> None."""
    m = re.match(r"(\d+)", choice)
    return int(m.group(1)) if m else None


# ----------------------------------------------------------------------
# Frame processing (shared by every camera source)
# ----------------------------------------------------------------------

def process_frame(frame, max_value, threshold_ratio, offset=(0, 0), display_max_px=DISPLAY_MAX_PX):
    """
    One camera frame -> display image + sub-pixel centroid.

    Coarse-to-fine so a 20 MP frame stays fast (~60 ms):
      1. Block-average down to ~display_max_px (also used for display).
      2. Estimate the background (median) and find the spot where the
         signal rises above  background + threshold * (peak - background).
      3. Return to FULL-resolution pixels inside that box and compute a
         background-subtracted weighted centroid there.
    Centroid coordinates are continuous full-sensor coordinates (ROI
    offset added, pixel i's center at i + 0.5), so a calibration stays
    valid when the ROI changes and the image, markers and axes agree.
    """
    h, w = frame.shape[:2]
    ox, oy = offset
    step = max(1, int(math.ceil(max(h, w) / float(display_max_px))))
    hh, ww = (h // step) * step, (w // step) * step
    small = frame[:hh, :ww].reshape(hh // step, step, ww // step, step).mean(axis=(1, 3), dtype=np.float32)
    display = np.clip(small * (255.0 / max_value), 0, 255).astype(np.uint8)

    background = float(np.median(small))
    peak_small = float(small.max())
    # Noise estimate: RMS about the background, excluding the brightest
    # 0.5% of pixels (where the spot is). A median-based estimate reads 0
    # when dark noise is clipped at zero (common with black level 0),
    # which let single noise spikes pass as a "spot".
    dev = (small - background).ravel()
    keep = max(1, int(dev.size * 0.995))
    dev = np.partition(dev, keep - 1)[:keep]
    noise = float(np.sqrt(np.mean(dev * dev)))
    result = {"display": display, "step": step, "width": w, "height": h, "offset": (ox, oy),
              "cx": None, "cy": None, "spot_px": None, "peak": peak_small,
              "background": background, "noise": noise, "saturated": False}

    # Detect/no-detect is decided on an image averaged over >= 9 sensor
    # pixels. A real return always covers several neighboring pixels; a
    # lone noise spike (very short exposure, no return) does not, and
    # averaging wipes it out. A big frame is already block-averaged by
    # step^2 >= 9; a small one (simulator, small ROI) gets a 3x3 mean.
    if step < 3:
        p = np.pad(small, 1, mode="edge")
        det = sum(p[dy:dy + small.shape[0], dx:dx + small.shape[1]]
                  for dy in range(3) for dx in range(3)) / 9.0
        det_bg = float(np.median(det))
        d = (det - det_bg).ravel()
        keep = max(1, int(d.size * 0.995))
        d = np.partition(d, keep - 1)[:keep]
        det_noise = float(np.sqrt(np.mean(d * d)))
    else:
        det, det_bg, det_noise = small, background, noise
    # Fixed floor is 3% of full scale for a single pixel; in a block-
    # averaged image a tight spot's light is spread over step^2 pixels,
    # so the floor scales down with it (otherwise tight spots on the
    # full 20 MP frame were missed).
    floor = 0.03 * max_value / (step * step if step >= 3 else 1)
    if float(det.max()) - det_bg < max(floor, 8.0 * det_noise):
        return result

    level = background + threshold_ratio * (peak_small - background)
    ys, xs = np.nonzero(small >= level)
    if ys.size == 0:
        return result

    y0 = max(0, (int(ys.min()) - 1) * step)
    y1 = min(h, (int(ys.max()) + 2) * step)
    x0 = max(0, (int(xs.min()) - 1) * step)
    x1 = min(w, (int(xs.max()) + 2) * step)

    roi = frame[y0:y1, x0:x1].astype(np.float32)
    full_peak = float(roi.max())
    result["peak"] = full_peak
    result["saturated"] = full_peak >= max_value
    full_level = background + threshold_ratio * (full_peak - background)
    weights = roi - full_level
    np.maximum(weights, 0, out=weights)
    total = float(weights.sum())
    if total <= 0:
        return result

    col_sums = weights.sum(axis=0, dtype=np.float64)
    row_sums = weights.sum(axis=1, dtype=np.float64)
    xi = np.arange(col_sums.size, dtype=np.float64)
    yi = np.arange(row_sums.size, dtype=np.float64)
    mx = float(col_sums @ xi) / total
    my = float(row_sums @ yi) / total
    result["cx"] = ox + x0 + mx + 0.5
    result["cy"] = oy + y0 + my + 0.5

    # Spot size (focus aid): RMS diameter (2 sigma) of the thresholded
    # spot. Relative measure — smaller means sharper focus.
    var_x = float(col_sums @ (xi - mx) ** 2) / total
    var_y = float(row_sums @ (yi - my) ** 2) / total
    result["spot_px"] = 2.0 * math.sqrt(max(0.0, 0.5 * (var_x + var_y)))
    return result


# ----------------------------------------------------------------------
# Camera sources
# ----------------------------------------------------------------------
# Every source has the same interface, so the GUI never cares which one
# is active:
#   connect(settings) / start() / disconnect() / is_alive()
#   get_latest() -> (frame_id, result dict)
#   set_exposure_ms / set_gain_db / set_black_level / set_frame_rate
#       (each returns the value the camera ACTUALLY applied)
#   reconfigure(pixel_format=, roi_size=, roi_center=, flip_x=, flip_y=)
#   limits dict, resulting_fps(), temperature_c(), auto_exposure_once()
#   latest_raw() -> full-resolution frame for saving

class SimulatedSource:
    """No-hardware stand-in: a drifting Gaussian spot with noise.
    Exposure, gain, black level and flip all affect the image the way
    they would on a real camera, so the controls can be exercised."""

    is_simulator = True
    is_emulated = False

    def __init__(self):
        self.width = self.height = FRAME_SIZE
        self.sensor_w = self.sensor_h = FRAME_SIZE
        self.offset = (0, 0)
        self.max_value = 255
        self.pixel_format = "Mono8"
        self.model_name = "Simulator"
        self.serial = "--"
        self.flip_x = self.flip_y = False
        self.threshold_ratio = 0.45
        self.forced_pixel = None     # Test Point mode: exact spot position
        self.last_error = None
        self.limits = {"exposure_ms": (EXPOSURE_MIN_MS, EXPOSURE_MAX_MS), "gain_db": (0.0, GAIN_MAX_DB),
                       "black_level": (0.0, BLACK_LEVEL_MAX), "frame_rate": (1.0, 30.0)}
        self._exposure_ms = EXPOSURE_DEFAULT_MS
        self._gain_db = 0.0
        self._black = 0.0
        self._fps = FRAME_RATE_DEFAULT
        self._frame_id = 0
        self._raw = None
        self.reset_walk()

    def reset_walk(self):
        self.true_x = FRAME_SIZE / 2.0
        self.true_y = FRAME_SIZE / 2.0

    # lifecycle (nothing to do)
    def connect(self, settings):
        self.reconfigure(flip_x=settings.get("flip_x", False), flip_y=settings.get("flip_y", False))

    def start(self):
        pass

    def disconnect(self):
        pass

    def is_alive(self):
        return True

    # settings
    def set_exposure_ms(self, ms):
        lo, hi = self.limits["exposure_ms"]
        self._exposure_ms = min(max(float(ms), lo), hi)
        return self._exposure_ms

    def get_exposure_ms(self):
        return self._exposure_ms

    def set_gain_db(self, db):
        lo, hi = self.limits["gain_db"]
        self._gain_db = min(max(float(db), lo), hi)
        return self._gain_db

    def set_black_level(self, value):
        lo, hi = self.limits["black_level"]
        self._black = min(max(float(value), lo), hi)
        return self._black

    def set_frame_rate(self, fps):
        lo, hi = self.limits["frame_rate"]
        self._fps = min(max(float(fps), lo), hi)
        return self._fps

    def resulting_fps(self):
        return None

    def temperature_c(self):
        return None

    def auto_exposure_once(self):
        """Pick the exposure that puts the spot peak at ~70% of full scale."""
        gain = 10 ** (self._gain_db / 20.0)
        target = 0.7 * self.max_value - SIM_BACKGROUND * gain - self._black
        per_ms = SIM_SPOT_PEAK * gain / SIM_REF_EXPOSURE_MS
        if per_ms > 0 and target > 0:
            self.set_exposure_ms(target / per_ms)
        return True

    def auto_exposure_busy(self):
        return False

    def auto_exposure_stop(self):
        pass

    def reconfigure(self, pixel_format=None, roi_size=None, roi_center=None, flip_x=None, flip_y=None):
        if flip_x is not None:
            self.flip_x = bool(flip_x)
        if flip_y is not None:
            self.flip_y = bool(flip_y)
        return None   # pixel format / ROI not applicable to the simulator

    def latest_raw(self):
        return self._raw

    # frames
    def get_latest(self):
        if self.forced_pixel is not None:
            cx, cy = self.forced_pixel
            self.true_x, self.true_y = cx, cy
        else:
            self.true_x = min(max(self.true_x + random.gauss(0, 0.07), 20), FRAME_SIZE - 20)
            self.true_y = min(max(self.true_y + random.gauss(0, 0.07), 20), FRAME_SIZE - 20)
            cx, cy = self.true_x, self.true_y
        gain = 10 ** (self._gain_db / 20.0)
        scale = self._exposure_ms / SIM_REF_EXPOSURE_MS
        yy, xx = np.mgrid[0:FRAME_SIZE, 0:FRAME_SIZE]
        sigma = 1.8
        # continuous coordinates: pixel i's center is at i + 0.5
        frame = SIM_SPOT_PEAK * scale * np.exp(-(((xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2)
                                                 / (2 * sigma ** 2)))
        frame = (frame + SIM_BACKGROUND * scale) * gain + self._black
        noise = 0.1 if self.forced_pixel is not None else 2.0
        frame += np.random.normal(0, noise * gain, frame.shape)
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        if self.flip_x:
            frame = frame[:, ::-1]
        if self.flip_y:
            frame = frame[::-1, :]
        self._raw = frame
        res = process_frame(frame, self.max_value, self.threshold_ratio)
        res["t"] = time.perf_counter()
        self._frame_id += 1
        return self._frame_id, res


class BaslerSource:
    """
    Live acquisition from the Basler camera (or pylon's emulator) via
    pypylon. Grabbing and the heavy per-frame math run on a background
    thread; the GUI just picks up finished results. Every camera
    feature access is wrapped, so a feature a particular camera or the
    emulator lacks degrades gracefully instead of crashing.
    """

    is_simulator = False

    def __init__(self, use_emulator=False):
        self.use_emulator = use_emulator
        self.pylon = None
        self.genicam = None
        self.cam = None
        self.width = self.sensor_w = CAMERA_WIDTH_PX
        self.height = self.sensor_h = CAMERA_HEIGHT_PX
        self.offset = (0, 0)
        self.max_value = 255
        self.pixel_format = "Mono8"
        self.model_name = ""
        self.serial = ""
        self.is_emulated = False
        self.flip_x = self.flip_y = False
        self.threshold_ratio = 0.45
        self.last_error = None
        self.limits = {}
        self.link_speed_bps = None       # USB link speed reported by the camera
        self.available_formats = []      # pixel formats this camera offers
        self.warnings = []               # bring-up problems worth telling the user
        self.emulator_spot_dir = None
        self._lock = threading.Lock()
        self._latest = None
        self._raw = None
        self._frame_id = 0
        self._thread = None
        self._stop = threading.Event()
        self._removed = False

    # ---------------- connection ----------------

    def _choose_device(self, devices):
        """Emulator vs real camera, then serial number (if set), then the
        a2A4504 model, then anything Basler."""
        if self.use_emulator:
            devices = [d for d in devices if d.GetDeviceClass() == "BaslerCamEmu"]
            if not devices:
                raise RuntimeError("pylon emulator not available.\nPYLON_CAMEMU was not set before pylon "
                                   "started — restart the app.")
            return devices[0]
        devices = [d for d in devices if d.GetDeviceClass() != "BaslerCamEmu"]
        if not devices:
            raise RuntimeError("No Basler camera found.\nCheck the USB 3 cable/port, close pylon Viewer, "
                               "then Reconnect.")
        if CAMERA_SERIAL:
            match = [d for d in devices if d.GetSerialNumber() == CAMERA_SERIAL]
            if not match:
                found = ", ".join(f"{d.GetModelName()} SN {d.GetSerialNumber()}" for d in devices)
                raise RuntimeError(f"Camera SN {CAMERA_SERIAL} (DUV_CAMERA_SERIAL) not found.\nFound: {found}")
            return match[0]
        return next((d for d in devices if CAMERA_MODEL in d.GetModelName()), devices[0])

    def connect(self, settings):
        from pypylon import pylon, genicam       # ImportError handled by the caller
        self.pylon, self.genicam = pylon, genicam

        tl = pylon.TlFactory.GetInstance()
        chosen = self._choose_device(list(tl.EnumerateDevices()))
        self.cam = pylon.InstantCamera(tl.CreateDevice(chosen))
        try:
            self.cam.Open()
        except Exception as exc:
            self.cam = None
            raise RuntimeError(f"Could not open {chosen.GetModelName()} — is pylon Viewer (or another "
                               f"copy of this app) using it?\n{exc}") from exc
        self.model_name = chosen.GetModelName()
        self.serial = chosen.GetSerialNumber()
        self.is_emulated = chosen.GetDeviceClass() == "BaslerCamEmu"
        self.cam.MaxNumBuffer.Value = 5
        self.warnings = []
        if not self.is_emulated and CAMERA_MODEL not in self.model_name:
            self.warnings.append(f"Expected {CAMERA_MODEL}, found {self.model_name}")

        # Metrology needs a linear, manual camera: no auto exposure or
        # gain changing the signal mid-measurement, gamma = 1, and no
        # on-camera image processing.
        for name, value in (("AcquisitionMode", "Continuous"), ("TriggerMode", "Off"),
                            ("ExposureAuto", "Off"), ("GainAuto", "Off"), ("Gamma", 1.0),
                            ("BinningHorizontal", 1), ("BinningVertical", 1),
                            ("TestImageSelector", "Off"), ("TestPattern", "Off"),
                            ("DeviceTemperatureSelector", "Sensor")):
            self._set(name, value)

        self.sensor_w = int(self._get("SensorWidth", None) or self._get("WidthMax", CAMERA_WIDTH_PX))
        self.sensor_h = int(self._get("SensorHeight", None) or self._get("HeightMax", CAMERA_HEIGHT_PX))
        self.available_formats = self._enum_values("PixelFormat")

        if self.is_emulated:
            # emulator: play back a synthetic return spot at full sensor
            # size (falls back to pylon's test image)
            self.emulator_spot_dir = ensure_emulator_spot_images(self.sensor_w, self.sensor_h)
            ok = (self.emulator_spot_dir is not None and self._set("ImageFilename", self.emulator_spot_dir)
                  and self._set("ImageFileMode", "On"))
            if not ok:
                self._set("TestImageSelector", "Testimage1")
        else:
            # USB link: on a USB 2 port the camera enumerates but can't stream
            self.link_speed_bps = self._get("DeviceLinkSpeed")
            if self.link_speed_bps is not None and float(self.link_speed_bps) < USB3_MIN_LINK_BPS:
                self.warnings.append("Camera is on a USB 2 link — use a USB 3 (blue/SS) port")
        self.limits = {
            "exposure_ms": self._range_scaled(("ExposureTime", "ExposureTimeAbs"), 1e-3,
                                              (EXPOSURE_MIN_MS, EXPOSURE_MAX_MS)),
            "gain_db": self._range(("Gain",), (0.0, GAIN_MAX_DB)),
            "black_level": self._range(("BlackLevel",), (0.0, BLACK_LEVEL_MAX)),
            "frame_rate": self._range(("AcquisitionFrameRate", "AcquisitionFrameRateAbs"),
                                      (1.0, CAMERA_MAX_FPS)),
        }
        # no point offering more than the camera can do (emulator reports 5000 fps)
        lo, hi = self.limits["frame_rate"]
        self.limits["frame_rate"] = (max(lo, 0.5), min(hi, 60.0))
        self.limits["exposure_ms"] = (self.limits["exposure_ms"][0], min(self.limits["exposure_ms"][1], 5000.0))

        self._apply_geometry(settings.get("pixel_format", "Mono8"), settings.get("roi_size"),
                             settings.get("roi_center"), settings.get("flip_x", False), settings.get("flip_y", False))

    def start(self):
        self._stop.clear()
        self.cam.StartGrabbing(self.pylon.GrabStrategy_LatestImageOnly)
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def _stop_grabbing(self):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None
        try:
            if self.cam is not None and self.cam.IsGrabbing():
                self.cam.StopGrabbing()
        except Exception:
            pass

    def disconnect(self):
        self._stop_grabbing()
        try:
            if self.cam is not None and self.cam.IsOpen():
                self.cam.Close()
        except Exception:
            pass
        self.cam = None

    def is_alive(self):
        if self.cam is None or self._removed:
            return False
        try:
            if self.cam.IsCameraDeviceRemoved():
                self._removed = True
                return False
        except Exception:
            pass
        return True

    # ---------------- feature access helpers ----------------

    def _node(self, name):
        try:
            return getattr(self.cam, name)
        except Exception:
            return None

    def _get(self, name, default=None):
        node = self._node(name)
        if node is None:
            return default
        try:
            return node.Value
        except Exception:
            return default

    def _set(self, name, value):
        node = self._node(name)
        if node is None:
            return False
        try:
            if not self.genicam.IsWritable(node):
                return False
            node.Value = value
            return True
        except Exception:
            return False

    def _range(self, names, fallback):
        for name in names:
            node = self._node(name)
            if node is None:
                continue
            try:
                return float(node.Min), float(node.Max)
            except Exception:
                continue
        return fallback

    def _range_scaled(self, names, scale, fallback):
        lo, hi = self._range(names, (None, None))
        if lo is None:
            return fallback
        return lo * scale, hi * scale

    def _set_float(self, names, value, scale=1.0):
        """Write the first writable of names (value in UI units / scale),
        clamped to that node's limits. Returns the applied value in UI
        units, read back from the camera, or None."""
        for name in names:
            node = self._node(name)
            if node is None:
                continue
            try:
                if not self.genicam.IsWritable(node):
                    continue
                raw = min(max(float(value) / scale, node.Min), node.Max)
                try:
                    node.Value = raw
                except Exception:
                    node.Value = int(round(raw))       # integer-type node
                return float(node.Value) * scale
            except Exception:
                continue
        return None

    def _enum_values(self, name):
        """Symbolic values of an enumeration feature that are available now."""
        node = self._node(name)
        if node is None:
            return []
        try:
            return [str(v) for v in node.Symbolics]
        except Exception:
            return []

    def _set_int_aligned(self, name, value):
        node = self._node(name)
        if node is None:
            return False
        try:
            inc = max(1, int(node.Inc))
            v = int(min(max(int(value), node.Min), node.Max))
            v -= (v - int(node.Min)) % inc
            node.Value = v
            return True
        except Exception:
            return False

    # ---------------- settings ----------------

    def set_exposure_ms(self, ms):
        return self._set_float(("ExposureTime", "ExposureTimeAbs"), ms, scale=1e-3)

    def get_exposure_ms(self):
        for name in ("ExposureTime", "ExposureTimeAbs"):
            v = self._get(name)
            if v is not None:
                return float(v) * 1e-3
        return None

    def set_gain_db(self, db):
        return self._set_float(("Gain",), db)

    def set_black_level(self, value):
        return self._set_float(("BlackLevel",), value)

    def set_frame_rate(self, fps):
        self._set("AcquisitionFrameRateEnable", True)
        return self._set_float(("AcquisitionFrameRate", "AcquisitionFrameRateAbs"), fps)

    def resulting_fps(self):
        for name in ("ResultingFrameRate", "ResultingFrameRateAbs"):
            v = self._get(name)
            if v is not None:
                try:
                    v = float(v)
                    if 0 < v < 1e6:
                        return v
                except Exception:
                    pass
        return None

    def temperature_c(self):
        v = self._get("DeviceTemperature")
        try:
            return float(v) if v is not None else None
        except Exception:
            return None

    def auto_exposure_once(self):
        return self._set("ExposureAuto", "Once")

    def auto_exposure_busy(self):
        return str(self._get("ExposureAuto", "Off")) == "Once"

    def auto_exposure_stop(self):
        self._set("ExposureAuto", "Off")

    def latest_raw(self):
        with self._lock:
            return self._raw

    # ---------------- geometry (needs grabbing stopped) ----------------

    def _apply_geometry(self, pixel_format, roi_size, roi_center, flip_x, flip_y):
        if pixel_format:
            for pf in PIXEL_FORMAT_FALLBACKS.get(pixel_format, [pixel_format]):
                if (not self.available_formats or pf in self.available_formats) and self._set("PixelFormat", pf):
                    break
        if flip_x is not None and self._set("ReverseX", bool(flip_x)):
            self.flip_x = bool(flip_x)
        if flip_y is not None and self._set("ReverseY", bool(flip_y)):
            self.flip_y = bool(flip_y)

        # ROI: start from full sensor (offsets to 0 first, so the full
        # width/height are allowed), then shrink + center if requested
        self._set("OffsetX", 0)
        self._set("OffsetY", 0)
        self._set_int_aligned("Width", self.sensor_w)
        self._set_int_aligned("Height", self.sensor_h)
        if roi_size:
            size_w = min(int(roi_size), self.sensor_w)
            size_h = min(int(roi_size), self.sensor_h)
            self._set_int_aligned("Width", size_w)
            self._set_int_aligned("Height", size_h)
            w = int(self._get("Width", size_w))
            h = int(self._get("Height", size_h))
            cx, cy = roi_center if roi_center else (self.sensor_w / 2.0, self.sensor_h / 2.0)
            self._set_int_aligned("OffsetX", min(max(cx - w / 2.0, 0), self.sensor_w - w))
            self._set_int_aligned("OffsetY", min(max(cy - h / 2.0, 0), self.sensor_h - h))

        self.width = int(self._get("Width", self.sensor_w))
        self.height = int(self._get("Height", self.sensor_h))
        self.offset = (int(self._get("OffsetX", 0)), int(self._get("OffsetY", 0)))
        self.pixel_format = str(self._get("PixelFormat", "Mono8"))
        bits = re.search(r"(\d+)", self.pixel_format)
        self.max_value = (2 ** int(bits.group(1)) - 1) if bits else 255

    def reconfigure(self, pixel_format=None, roi_size=None, roi_center=None, flip_x=None, flip_y=None):
        """Change settings that the camera only accepts while stopped.
        Returns None on success or an error string."""
        was_grabbing = self._thread is not None
        self._stop_grabbing()
        try:
            self._apply_geometry(pixel_format, roi_size, roi_center, flip_x, flip_y)
            with self._lock:
                self._latest = None
        except Exception as exc:
            return str(exc)
        finally:
            if was_grabbing:
                self.start()
        return None

    # ---------------- acquisition thread ----------------

    def _grab_loop(self):
        pylon = self.pylon
        while not self._stop.is_set():
            res = None
            try:
                if self.cam is None or not self.cam.IsGrabbing():
                    break
                res = self.cam.RetrieveResult(1000, pylon.TimeoutHandling_Return)
                if res is None or not res.IsValid():
                    continue
                if not res.GrabSucceeded():
                    self.last_error = f"Grab failed: {res.GetErrorDescription()}"
                    continue
                frame = res.GetArray()
            except Exception as exc:
                self.last_error = str(exc)
                try:
                    if self.cam is not None and self.cam.IsCameraDeviceRemoved():
                        self._removed = True
                        break
                except Exception:
                    pass
                time.sleep(0.1)
                continue
            finally:
                try:
                    if res is not None and res.IsValid():
                        res.Release()
                except Exception:
                    pass

            try:
                processed = process_frame(frame, self.max_value, self.threshold_ratio, self.offset)
            except Exception as exc:
                self.last_error = f"Processing error: {exc}"
                continue
            processed["t"] = time.perf_counter()
            with self._lock:
                self._latest = processed
                self._raw = frame
                self._frame_id += 1
            self.last_error = None

    def get_latest(self):
        with self._lock:
            return self._frame_id, self._latest



# ----------------------------------------------------------------------
# Theme-controlled widgets
# ----------------------------------------------------------------------
# macOS draws native buttons, comboboxes, spinbox arrows and focus rings
# itself (blue, ignoring the app's colors) — unreadable through blue-
# blocking laser glasses. Everything interactive here is built from
# plain Labels/Entries the app colors completely.

class ThemedDropdown(tk.Frame):
    """Replacement for ttk.Combobox. Fires <<ComboboxSelected>>."""

    ARROW = "  ▾"

    def __init__(self, parent, textvariable, values, theme_getter, width=None, font=("Arial", 10)):
        super().__init__(parent, bd=0, highlightthickness=0)
        kw = dict(font=font, relief=tk.FLAT, bd=0, highlightthickness=0, padx=6, pady=3,
                  cursor="hand2", anchor="w")
        if width is not None:
            kw["width"] = width
        self.label = tk.Label(self, **kw)
        self.label.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        self._var = textvariable
        self._values = list(values)
        self._theme_getter = theme_getter
        self._font = font
        self._popup = None
        self._enabled = True
        self._var.trace_add("write", lambda *_: self._refresh_text())
        self.label.bind("<Button-1>", lambda e: self._toggle_popup())
        # one permanent window-wide handler closes the list on outside clicks
        self.winfo_toplevel().bind("<Button-1>", self._on_outside_click, add="+")
        self._refresh_text()

    def __getitem__(self, key):
        if key == "values":
            return tuple(self._values)
        return super().__getitem__(key)

    def __setitem__(self, key, value):
        if key == "values":
            self._values = list(value)
            return
        super().__setitem__(key, value)

    @staticmethod
    def _fmt(value):
        if isinstance(value, float) and value.is_integer():
            return str(int(value))
        return str(value)

    def _refresh_text(self):
        try:
            current = self._var.get()
        except tk.TclError:
            current = ""
        self.label.configure(text=self._fmt(current) + self.ARROW)

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)
        self.apply_theme(self._theme_getter())

    def apply_theme(self, t):
        self.configure(bg=t["border"])
        self.label.configure(bg=t["entry_bg"], fg=t["entry_fg"] if self._enabled else t["neutral"])

    def _toggle_popup(self):
        if self._popup is not None:
            self._close_popup()
            return
        if not self._values or not self._enabled:
            return
        t = self._theme_getter()
        pop = tk.Toplevel(self)
        pop.withdraw()
        pop.overrideredirect(True)
        try:
            pop.transient(self.winfo_toplevel())
            pop.attributes("-topmost", True)
        except tk.TclError:
            pass
        pop.configure(bg=t["border"])
        inner = tk.Frame(pop, bg=t["entry_bg"])
        inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        try:
            current = self._var.get()
        except tk.TclError:
            current = None
        for value in self._values:
            selected = (value == current)
            nb, nf = (t["fg"], t["entry_bg"]) if selected else (t["entry_bg"], t["entry_fg"])
            item = tk.Label(inner, text=self._fmt(value), font=self._font, anchor="w",
                            padx=8, pady=3, bg=nb, fg=nf, cursor="hand2")
            item.pack(fill=tk.X)
            item.bind("<Enter>", lambda e, w=item: w.configure(bg=t["fg"], fg=t["entry_bg"]))
            item.bind("<Leave>", lambda e, w=item, b=nb, f=nf: w.configure(bg=b, fg=f))
            item.bind("<ButtonRelease-1>", lambda e, v=value: self._choose(v))
        self.update_idletasks()
        pop.geometry(f"+{self.winfo_rootx()}+{self.winfo_rooty() + self.winfo_height()}")
        pop.minsize(self.winfo_width(), 1)
        pop.deiconify()
        pop.lift()
        self._popup = pop
        pop.bind("<Escape>", lambda e: self._close_popup())
        pop.focus_force()

    def _on_outside_click(self, event):
        if self._popup is None:
            return
        w = event.widget
        if w is self or w is self.label or str(w).startswith(str(self._popup)):
            return
        self._close_popup()

    def _close_popup(self):
        if self._popup is not None:
            try:
                self._popup.destroy()
            except tk.TclError:
                pass
            self._popup = None

    def _choose(self, value):
        self._close_popup()
        self._var.set(value)
        self.event_generate("<<ComboboxSelected>>")


class NumberField(tk.Frame):
    """
    Numeric entry with themed − / + step buttons (replaces tk.Spinbox,
    whose arrows macOS draws natively). The value is committed on
    Return, focus-out, the step buttons, or Up/Down arrow keys; invalid
    or out-of-range input snaps back / clamps instead of raising.
    on_commit(value) may return the value actually applied (e.g. what
    the camera accepted), which is then shown.
    """

    def __init__(self, parent, value, lo, hi, step, decimals, theme_getter, mono_font,
                 width=7, on_commit=None):
        super().__init__(parent, bd=0, highlightthickness=0)
        self.lo, self.hi, self.step, self.decimals = float(lo), float(hi), float(step), int(decimals)
        self._theme_getter = theme_getter
        self.on_commit = on_commit
        self._enabled = True
        self._value = self._clamp(value)
        self._text = tk.StringVar(value=self._fmt(self._value))
        self.minus = tk.Label(self, text="−", font=("Arial", 10, "bold"), width=2, cursor="hand2")
        self.entry = tk.Entry(self, textvariable=self._text, width=width, font=mono_font,
                              justify="right", relief=tk.FLAT, bd=2, highlightthickness=1)
        self.plus = tk.Label(self, text="+", font=("Arial", 10, "bold"), width=2, cursor="hand2")
        self.minus.pack(side=tk.LEFT, fill=tk.Y)
        self.entry.pack(side=tk.LEFT, fill=tk.Y, padx=1)
        self.plus.pack(side=tk.LEFT, fill=tk.Y)
        self.minus.bind("<Button-1>", lambda e: self._nudge(-1))
        self.plus.bind("<Button-1>", lambda e: self._nudge(+1))
        self.entry.bind("<Return>", lambda e: self._commit_text())
        self.entry.bind("<KP_Enter>", lambda e: self._commit_text())
        self.entry.bind("<FocusOut>", lambda e: self._commit_text())
        self.entry.bind("<Up>", lambda e: self._nudge(+1))
        self.entry.bind("<Down>", lambda e: self._nudge(-1))

    def _fmt(self, v):
        return f"{v:.{self.decimals}f}"

    def _clamp(self, v):
        return min(max(float(v), self.lo), self.hi)

    def get(self):
        return self._value

    def set(self, value, notify=False):
        self._value = self._clamp(value)
        self._text.set(self._fmt(self._value))
        if notify:
            self._notify()

    def set_range(self, lo, hi):
        self.lo, self.hi = float(lo), float(hi)
        self.set(self._value)

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)
        self.entry.configure(state="normal" if enabled else "disabled")
        self.apply_theme(self._theme_getter())

    def _notify(self):
        if self.on_commit is not None:
            applied = self.on_commit(self._value)
            if applied is not None:
                self._value = float(applied)
                self._text.set(self._fmt(self._value))

    def _commit_text(self):
        if not self._enabled:
            return
        try:
            v = float(self._text.get().strip().replace(",", ""))
        except ValueError:
            self._text.set(self._fmt(self._value))     # invalid: restore last good value
            return
        v = self._clamp(v)
        changed = abs(v - self._value) > 1e-12
        self._value = v
        self._text.set(self._fmt(v))
        if changed:
            self._notify()

    def _nudge(self, direction):
        if not self._enabled:
            return "break"
        self._commit_text()
        self.set(self._value + direction * self.step, notify=True)
        return "break"

    def apply_theme(self, t):
        self.configure(bg=t["panel_bg"])
        fg = t["entry_fg"] if self._enabled else t["neutral"]
        self.entry.configure(bg=t["entry_bg"], fg=fg, insertbackground=t["entry_fg"],
                             disabledbackground=t["entry_bg"], disabledforeground=t["neutral"],
                             highlightbackground=t["border"], highlightcolor=t["fg"])
        for b in (self.minus, self.plus):
            b.configure(bg=t["entry_bg"], fg=fg, highlightthickness=0)


class SliderField:
    """
    Label + slider + NumberField for one camera setting. The slider can
    be logarithmic (exposure spans 20 us .. 1 s). The callback receives
    the requested value and returns what the camera actually applied,
    which both the slider and the field then show.
    """

    STEPS = 1000

    def __init__(self, parent, row, label, lo, hi, value, decimals, step, unit, log, theme_getter,
                 mono_font, on_change, registry):
        self.lo, self.hi, self.log = float(lo), float(hi), bool(log)
        self.on_change = on_change
        self._busy = False
        self._enabled = True
        self.tag = tk.Label(parent, text=label, font=("Arial", 10), anchor="w")
        self.tag.grid(row=row, column=0, sticky="w", padx=(4, 2), pady=1)
        self._pos = tk.DoubleVar(value=self._to_pos(value))
        self.scale = tk.Scale(parent, from_=0, to=self.STEPS, orient=tk.HORIZONTAL, variable=self._pos,
                              showvalue=False, resolution=1, command=lambda e: self._on_slider(),
                              highlightthickness=0, bd=1, sliderlength=18, width=12)
        self.scale.grid(row=row, column=1, sticky="ew", padx=2, pady=1)
        self.field = NumberField(parent, value, lo, hi, step, decimals, theme_getter, mono_font,
                                 width=8, on_commit=self._on_field)
        self.field.grid(row=row, column=2, sticky="e", padx=2, pady=1)
        self.unit = tk.Label(parent, text=unit, font=("Arial", 9), anchor="w", width=4)
        self.unit.grid(row=row, column=3, sticky="w", padx=(0, 4))
        registry["labels"].extend([(self.tag, "text"), (self.unit, "text")])
        registry["scales"].append(self.scale)
        registry["custom"].append(self.field)

    def _to_pos(self, v):
        v = min(max(float(v), self.lo), self.hi)
        if self.log and self.lo > 0:
            return self.STEPS * math.log(v / self.lo) / math.log(self.hi / self.lo)
        return self.STEPS * (v - self.lo) / max(1e-12, self.hi - self.lo)

    def _from_pos(self, p):
        p = min(max(float(p), 0), self.STEPS) / self.STEPS
        if self.log and self.lo > 0:
            return self.lo * (self.hi / self.lo) ** p
        return self.lo + p * (self.hi - self.lo)

    def _apply(self, requested):
        applied = self.on_change(requested) if self.on_change else requested
        if applied is None:
            applied = requested
        self.set(applied)
        return applied

    def _on_slider(self):
        if self._busy or not self._enabled:
            return
        # Tk fires the slider callback (at idle) even when the position was
        # set from code; ignore it unless the user actually moved the
        # slider, or a typed exact value would be replaced by the nearest
        # slider step
        if abs(self._pos.get() - self._to_pos(self.field.get())) < 0.75:
            return
        self._apply(self._from_pos(self._pos.get()))

    def _on_field(self, value):
        if self._busy:
            return None
        return self._apply(value)

    def get(self):
        return self.field.get()

    def set(self, value):
        self._busy = True
        try:
            self.field.set(value)
            self._pos.set(self._to_pos(value))
        finally:
            self._busy = False

    def set_range(self, lo, hi):
        if hi <= lo:
            return
        self.lo, self.hi = float(lo), float(hi)
        self.field.set_range(lo, hi)
        self.set(self.field.get())

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)
        self.scale.configure(state="normal" if enabled else "disabled")
        self.field.set_enabled(enabled)



# ----------------------------------------------------------------------
# Main application
# ----------------------------------------------------------------------

class DUVAutocollimatorGUI:

    # ================================================================
    # Setup
    # ================================================================

    def __init__(self, root):
        self.root = root
        self.root.title("DUV Autocollimator Control")
        self._fit_window_to_screen()

        self.theme_name = DEFAULT_THEME
        self.theme = THEMES[self.theme_name]
        self.mono = pick_font_family(root, ["SF Mono", "Menlo", "Consolas", "DejaVu Sans Mono",
                                            "Liberation Mono", "Courier New", "Courier"], "TkFixedFont")
        self.mono9 = (self.mono, 10, "bold")

        self.running = True
        self.paused = False

        # ---- mode / camera source ----
        self.mode = DEFAULT_MODE
        self.mode_var = tk.StringVar(value=DEFAULT_MODE)
        self.source = None
        self.source_var = tk.StringVar(value=DEFAULT_SOURCE)
        self.source_frame_id = -1
        self.camera_status_var = tk.StringVar(value="Not connected")
        self._camera_status_base = "Not connected"
        self._status_note = None
        self.cam_info_var = tk.StringVar(value="")
        self.pixel_format_var = tk.StringVar(value="Mono8")
        self.roi_var = tk.StringVar(value=ROI_CHOICES[0])
        self.flip_x_var = tk.BooleanVar(value=False)
        self.flip_y_var = tk.BooleanVar(value=False)
        self.tab_var = tk.StringVar(value="Camera")
        self._disabled_chips = set()
        self._auto_exp_after_id = None

        # ---- measurement / display settings ----
        self.units_var = tk.StringVar(value=DATA_UNITS[0])
        self.rate_var = tk.DoubleVar(value=10.0)
        self.serial_ports = list_serial_ports()
        self.com_var = tk.StringVar(value=self.serial_ports[0])
        self.theme_var = tk.StringVar(value=self.theme_name)
        self.test_mode_var = tk.BooleanVar(value=False)

        # ---- profiles (persisted to ~/.duv_autocollimator/profiles.json) ----
        self.system_name_var = tk.StringVar(value="")
        self.profile_var = tk.StringVar(value="")
        self.system_profiles = self._load_profiles_file()

        # ---- capture / logging ----
        self.capture_dir_var = tk.StringVar(value=os.path.join(os.path.expanduser("~"), "Desktop"))
        self.capture_status_var = tk.StringVar(value="")
        self.log_var = tk.BooleanVar(value=False)
        self._csv_file = None
        self._csv_writer = None
        self._csv_rows = 0
        self._csv_t0 = None
        self._recording = False
        self._recording_frames = []
        self._recording_start_time = None
        self._recording_after_id = None
        self._recording_capture_error = None
        self._recording_mode = None
        self._ffmpeg_proc = None
        self._recording_output_path = None

        # ---- tolerance alert ----
        self.in_tolerance = None       # None = no signal
        self._alert_wav_out = None
        self._alert_wav_in = None
        self._ensure_alert_sounds()

        # ---- calibration (sensor-pixel coordinates of the zero point) ----
        self.sensor_w = self.sensor_h = CAMERA_WIDTH_PX
        self.frame_w = self.frame_h = CAMERA_WIDTH_PX     # current image (ROI) size
        self.display_offset = (0, 0)
        self.bias_x = self.sensor_w / 2.0
        self.bias_y = self.sensor_h / 2.0
        self.calibrated = False
        self._cal_key = None
        self.test_point_origin_x = FRAME_SIZE / 2.0
        self.test_point_origin_y = FRAME_SIZE / 2.0

        # ---- live values ----
        self.sample_index = deque(maxlen=CHART_HISTORY)
        self.az_hist = deque(maxlen=CHART_HISTORY)
        self.el_hist = deque(maxlen=CHART_HISTORY)
        self._n = 0
        self._frame_timestamps = deque(maxlen=15)
        self.measured_fps = 0.0
        self.last_az = self.last_el = None
        self.last_cx = self.last_cy = None
        self.last_signal = 0
        self.last_peak_raw = 0
        self.last_background = 0.0
        self.last_saturated = False
        self.last_spot_px = None
        self.last_display_frame = None
        self.display_step = 1
        self.peak_max = 255
        self.exposure_actual_ms = None
        self.temp_c = None
        self.camera_max_fps = None
        self._last_poll = 0.0

        # ---- theme registries (filled while building) ----
        self.reg = {k: [] for k in ("frames", "labels", "labelframes", "entries", "scales", "checks",
                                    "radios", "buttons", "canvases", "dropdowns", "custom")}

        self._build_ui()
        for var in (self.test_mode_var, self.mode_var, self.source_var, self.pixel_format_var,
                    self.flip_x_var, self.flip_y_var, self.tab_var, self.log_var):
            var.trace_add("write", self._refresh_toggle_colors)
        self._last_session_json = None
        self._last_autosave = time.perf_counter()
        session = read_json(SESSION_FILE)
        self._restore_session(session)        # every setting from last time
        self.theme_combo["values"] = list(THEMES.keys())
        self.profile_combo["values"] = sorted(self.system_profiles)
        self._apply_theme(self.theme_name)
        self._show_tab(self.tab_var.get() if self.tab_var.get() in self.pages else "Camera")
        self._update_source_controls()
        self._update_plate_scale_label()
        self._configure_feed_axes()
        self._reset_measurement()
        self._update_cal_label()

        # reconnect to the source used last time (the real camera on first
        # launch) quietly; if it isn't there, the Camera tab says so
        start_source = self.source_var.get() if self.source_var.get() in SOURCES else DEFAULT_SOURCE
        self._startup_after_id = self.root.after(150, lambda: self._select_source(start_source, force=True,
                                                                                interactive=False))
        self._loop_after_id = self.root.after(250, self._update_loop)

    def _fit_window_to_screen(self):
        """Size the window to the actual screen. A fixed 1500 px window
        is wider than a 13-14" MacBook screen, which pushed the right
        side off-screen."""
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        w = max(1000, min(1500, sw - 20))
        h = max(680, min(920, sh - 80))
        self.root.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 3)}")
        self.root.minsize(min(1200, w), min(720, h))

    # ---------------- widget factories ----------------

    def _label(self, parent, text="", role="text", **kw):
        kw.setdefault("font", ("Arial", 10))
        lbl = tk.Label(parent, text=text, **kw)
        self.reg["labels"].append((lbl, role))
        return lbl

    def _frame(self, parent, **kw):
        f = tk.Frame(parent, **kw)
        self.reg["frames"].append((f, "panel"))
        return f

    def _labelframe(self, parent, text):
        lf = tk.LabelFrame(parent, text=text, font=("Arial", 10, "bold"), bd=1, relief=tk.GROOVE)
        self.reg["labelframes"].append(lf)
        return lf

    def _make_dropdown(self, parent, variable, values, width=None, font=("Arial", 10)):
        dd = ThemedDropdown(parent, variable, values, lambda: self.theme, width=width, font=font)
        self.reg["dropdowns"].append(dd)
        return dd

    def _make_number(self, parent, value, lo, hi, step, decimals, width=7, on_commit=None):
        nf = NumberField(parent, value, lo, hi, step, decimals, lambda: self.theme, self.mono9,
                         width=width, on_commit=on_commit)
        self.reg["custom"].append(nf)
        return nf

    def _make_chip(self, parent, text, var, value=None, command=None, kind="radio"):
        """Toggle chip (radio or check) drawn by us — no native glyphs."""
        chip = tk.Label(parent, text=text, font=("Arial", 10), relief=tk.RIDGE, borderwidth=1,
                        padx=7, pady=2, cursor="hand2")

        def _click(_e):
            if chip in self._disabled_chips:
                return
            if command is not None:
                command()
            elif kind == "check":
                var.set(not var.get())
            else:
                var.set(value)
        chip.bind("<Button-1>", _click)
        if kind == "check":
            self.reg["checks"].append((chip, var, text))
        else:
            self.reg["radios"].append((chip, var, value, text))
        return chip

    def _make_button(self, parent, text, command, font=("Arial", 10), width=None):
        """Label-based button (macOS ignores colors on tk.Button)."""
        kw = dict(font=font, relief=tk.RAISED, borderwidth=2, padx=8, pady=3, cursor="hand2")
        if width is not None:
            kw["width"] = width
        lbl = tk.Label(parent, text=text, **kw)

        def _press(_e):
            lbl.configure(relief=tk.SUNKEN)

        def _release(e):
            lbl.configure(relief=tk.RAISED)
            if 0 <= e.x <= lbl.winfo_width() and 0 <= e.y <= lbl.winfo_height():
                command()
        lbl.bind("<ButtonPress-1>", _press)
        lbl.bind("<ButtonRelease-1>", _release)
        self.reg["buttons"].append(lbl)
        return lbl

    def _make_canvas(self, fig, parent, min_w, min_h):
        """Matplotlib canvas that requests only a small size, so the
        layout gives it whatever room is LEFT after the controls."""
        canvas = FigureCanvasTkAgg(fig, master=parent)
        widget = canvas.get_tk_widget()
        widget.configure(width=min_w, height=min_h, highlightthickness=0)
        return canvas, widget

    # ---------------- layout ----------------

    def _build_ui(self):
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(2, weight=1)

        # row 0: theme + mode
        top = self._frame(self.root)
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(6, 0))
        self._label(top, "Display Theme:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        self.theme_combo = self._make_dropdown(top, self.theme_var, [], width=32)
        self.theme_combo.pack(side=tk.LEFT)
        self.theme_combo.bind("<<ComboboxSelected>>", lambda e: self._apply_theme(self.theme_var.get()))
        self._make_button(top, "Customize Colors…", self._open_color_customizer).pack(side=tk.LEFT, padx=(8, 0))
        self._label(top, "Mode:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(24, 6))
        for m in MODES:
            self._make_chip(top, m, self.mode_var, m, command=lambda m=m: self._set_mode(m)).pack(
                side=tk.LEFT, padx=(0, 4))

        # row 1: system profile
        prof = self._frame(self.root)
        prof.grid(row=1, column=0, sticky="ew", padx=8, pady=(6, 0))
        self._label(prof, "System Name:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        self.system_name_entry = tk.Entry(prof, textvariable=self.system_name_var, font=("Arial", 10),
                                          width=16, relief=tk.FLAT, bd=2, highlightthickness=1)
        self.system_name_entry.pack(side=tk.LEFT)
        self.reg["entries"].append(self.system_name_entry)
        self._label(prof, "Detection Threshold (%):", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(14, 6))
        self.threshold_field = self._make_number(prof, 45, 1, 99, 1, 0, width=4)
        self.threshold_field.pack(side=tk.LEFT)
        self._label(prof, "Saved Profiles:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(14, 6))
        self.profile_combo = self._make_dropdown(prof, self.profile_var, [], width=18)
        self.profile_combo.pack(side=tk.LEFT)
        self.profile_combo.bind("<<ComboboxSelected>>", self._load_system_profile)
        self._make_button(prof, "Save Profile", self._save_system_profile).pack(side=tk.LEFT, padx=(8, 0))
        self._make_button(prof, "Remove", self._remove_system_profile).pack(side=tk.LEFT, padx=(4, 0))
        self._make_button(prof, "Export…", self._export_profiles).pack(side=tk.LEFT, padx=(12, 0))
        self._make_button(prof, "Import…", self._import_profiles).pack(side=tk.LEFT, padx=(4, 0))

        # row 2: three columns
        outer = self._frame(self.root)
        outer.grid(row=2, column=0, sticky="nsew", padx=8, pady=8)
        outer.grid_rowconfigure(0, weight=1)
        # Column widths are a FIXED share of the window (uniform group +
        # weights) and the column frames don't propagate their contents'
        # size. So no label text, status message, mode or camera source
        # can ever widen/narrow a column and shove the plots around.
        outer.grid_columnconfigure(0, weight=10, minsize=360, uniform="cols")
        outer.grid_columnconfigure(1, weight=9, minsize=300, uniform="cols")
        outer.grid_columnconfigure(2, weight=11, minsize=340, uniform="cols")

        left = self._frame(outer)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        middle = self._frame(outer)
        middle.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        right = self._frame(outer)
        right.grid(row=0, column=2, sticky="nsew")
        for col in (left, middle, right):
            col.grid_propagate(False)

        self._build_left(left)
        self._build_middle(middle)
        self._build_right(right)

    # ---------------- LEFT: image/graph + camera tabs ----------------

    def _build_left(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=1)

        self.feed_title_label = self._label(parent, "", font=("Arial", 12, "bold"), anchor="w", width=1)
        self.feed_title_label.grid(row=0, column=0, sticky="ew")

        self.cam_fig = Figure(figsize=(4.4, 4.4), dpi=90)
        self.cam_ax = self.cam_fig.add_axes([0.13, 0.10, 0.84, 0.86])
        self.cam_canvas, cam_widget = self._make_canvas(self.cam_fig, parent, 380, 200)
        cam_widget.grid(row=1, column=0, sticky="nsew", pady=(2, 6))
        self._init_feed_artists()
        # identical square plot box in every mode and for every source
        pin_square_axes(self.cam_fig, self.cam_ax, FEED_MARGINS)

        # tab chips
        tabs = self._frame(parent)
        tabs.grid(row=2, column=0, sticky="ew")
        self.tab_chips = {}
        for name in ("Camera", "Settings", "Test Point"):
            chip = self._make_chip(tabs, name, self.tab_var, name, command=lambda n=name: self._show_tab(n))
            chip.pack(side=tk.LEFT, padx=(0, 4))
            self.tab_chips[name] = chip

        # page container: one fixed-height cell, all pages stacked in it.
        # Its height is set from the TALLEST page after building, so no
        # page can ever be cut off and switching never moves anything.
        self.page_host = self._frame(parent, bd=1, relief=tk.GROOVE)
        self.page_host.grid(row=3, column=0, sticky="ew", pady=(3, 0))
        self.page_host.grid_columnconfigure(0, weight=1)
        self.page_host.grid_rowconfigure(0, weight=1)
        self.pages = {}
        for name, builder in (("Camera", self._build_camera_page), ("Settings", self._build_settings_page),
                              ("Test Point", self._build_test_page)):
            page = self._frame(self.page_host)
            page.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
            builder(page)
            self.pages[name] = page
        self.page_host.update_idletasks()
        tallest = max(p.winfo_reqheight() for p in self.pages.values())
        self.page_host.grid_propagate(False)
        self.page_host.configure(height=tallest + 16, width=380)

    def _build_camera_page(self, page):
        page.grid_columnconfigure(0, weight=1)
        row = self._frame(page)
        row.grid(row=0, column=0, sticky="ew")
        self._label(row, "Source:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        for s in SOURCES:
            self._make_chip(row, s, self.source_var, s, command=lambda s=s: self._select_source(s)).pack(
                side=tk.LEFT, padx=(0, 4))

        # fixed-height status box: changing messages never move anything
        self.status_label = self._label(page, "", role="readout", textvariable=self.camera_status_var,
                                        font=(self.mono, 9), anchor="nw", justify="left", width=1,
                                        height=STATUS_LINES, relief=tk.SUNKEN, padx=4, pady=2)
        self.status_label.grid(row=1, column=0, sticky="ew", pady=(6, 4))
        self.status_label.bind("<Configure>", lambda e: self.status_label.configure(
            wraplength=max(100, e.width - 10)))

        btns = self._frame(page)
        btns.grid(row=2, column=0, sticky="ew")
        self._make_button(btns, "Reconnect", lambda: self._select_source(self.source_var.get(), force=True)).pack(
            side=tk.LEFT)
        self._make_button(btns, "Disconnect", self._on_disconnect_clicked).pack(side=tk.LEFT, padx=(4, 0))
        self.auto_exp_btn = self._make_button(btns, "Auto Exposure", self._auto_exposure)
        self.auto_exp_btn.pack(side=tk.LEFT, padx=(4, 0))

        self._label(page, "", textvariable=self.cam_info_var, font=("Arial", 9), anchor="w",
                    justify="left", width=1).grid(row=3, column=0, sticky="ew", pady=(6, 0))
        self._label(page, f"{CAMERA_DISPLAY_NAME} · {CAMERA_SENSOR} · {CAMERA_WIDTH_PX}×"
                          f"{CAMERA_HEIGHT_PX} px · {CAMERA_PIXEL_UM} µm · global shutter",
                    font=("Arial", 9), anchor="w", width=1).grid(row=4, column=0, sticky="ew")

    def _build_settings_page(self, page):
        page.grid_columnconfigure(1, weight=1)
        reg = {"labels": self.reg["labels"], "scales": self.reg["scales"], "custom": self.reg["custom"]}
        mk = lambda row, label, lo, hi, val, dec, step, unit, log, cb: SliderField(  # noqa: E731
            page, row, label, lo, hi, val, dec, step, unit, log, lambda: self.theme, self.mono9, cb, reg)
        self.exposure_ctl = mk(0, "Exposure", EXPOSURE_MIN_MS, EXPOSURE_MAX_MS, EXPOSURE_DEFAULT_MS, 3, 0.1,
                               "ms", True, self._on_exposure_change)
        self.gain_ctl = mk(1, "Gain", 0.0, GAIN_MAX_DB, 0.0, 1, 0.5, "dB", False, self._on_gain_change)
        self.black_ctl = mk(2, "Black Level", 0.0, BLACK_LEVEL_MAX, 0.0, 1, 1.0, "DN", False,
                            self._on_black_change)
        self.fps_ctl = mk(3, "Frame Rate", 1.0, CAMERA_MAX_FPS, FRAME_RATE_DEFAULT, 1, 1.0, "fps", False,
                          self._on_fps_change)

        r = self._frame(page)
        r.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(6, 2), padx=4)
        self._label(r, "Bit Depth", anchor="w", width=9).pack(side=tk.LEFT)
        self.format_chips = []
        for pf, text in PIXEL_FORMATS:
            chip = self._make_chip(r, text, self.pixel_format_var, pf, command=lambda pf=pf: self._select_pixel_format(pf))
            chip.pack(side=tk.LEFT, padx=(0, 4))
            self.format_chips.append(chip)
        self._label(r, "ROI", anchor="e").pack(side=tk.LEFT, padx=(12, 4))
        self.roi_combo = self._make_dropdown(r, self.roi_var, ROI_CHOICES, width=11)
        self.roi_combo.pack(side=tk.LEFT)
        self.roi_combo.bind("<<ComboboxSelected>>", lambda e: self._apply_geometry_change(roi=True))

        r2 = self._frame(page)
        r2.grid(row=5, column=0, columnspan=4, sticky="ew", pady=2, padx=4)
        self._label(r2, "Flip Image", anchor="w", width=9).pack(side=tk.LEFT)
        self.flip_chips = []
        for text, var in (("Flip X", self.flip_x_var), ("Flip Y", self.flip_y_var)):
            chip = self._make_chip(r2, text, var, kind="check",
                                   command=lambda v=var: self._toggle_flip(v))
            chip.pack(side=tk.LEFT, padx=(0, 4))
            self.flip_chips.append(chip)

        r3 = self._frame(page)
        r3.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(4, 0), padx=4)
        self._label(r3, "Focal Len.", anchor="w", width=9).pack(side=tk.LEFT)
        self.focal_field = self._make_number(r3, COLLIMATOR_FOCAL_MM_DEFAULT, 10, 5000, 1, 1, width=7,
                                             on_commit=lambda v: self._update_plate_scale_label())
        self.focal_field.pack(side=tk.LEFT)
        self._label(r3, "mm").pack(side=tk.LEFT, padx=(3, 0))
        self.plate_scale_label = self._label(r3, "", font=self.mono9, anchor="e")
        self.plate_scale_label.pack(side=tk.RIGHT)

    def _build_test_page(self, page):
        page.grid_columnconfigure(1, weight=1)
        self.test_note = self._label(page, "", font=("Arial", 9), anchor="w", justify="left", wraplength=360)
        self.test_note.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        self.test_mode_chip = self._make_chip(page, "Test Point Mode", self.test_mode_var, kind="check")
        self.test_mode_chip.grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 6))
        self._label(page, "Target AZ (arc-sec)", anchor="w").grid(row=2, column=0, sticky="w", pady=2)
        self.test_az_field = self._make_number(page, 0, -1000, 1000, 1, 1, width=8)
        self.test_az_field.grid(row=2, column=1, sticky="e", pady=2)
        self._label(page, "Target EL (arc-sec)", anchor="w").grid(row=3, column=0, sticky="w", pady=2)
        self.test_el_field = self._make_number(page, 0, -1000, 1000, 1, 1, width=8)
        self.test_el_field.grid(row=3, column=1, sticky="e", pady=2)

    # ---------------- MIDDLE: readout, stats, strip charts ----------------

    def _build_middle(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(5, weight=1)
        parent.grid_rowconfigure(6, weight=1)

        # AZ/EL readout. Colored by tolerance status, not by theme. The
        # digits' font size is fitted to the available width (see
        # _fit_readout), so the numbers can never be cut off.
        self.readout_font = tkfont.Font(family=self.mono, size=34, weight="bold")
        self.readout_tag_font = tkfont.Font(family="Arial", size=28, weight="bold")
        self.readout_frame = tk.Frame(parent, width=1, height=2 * self.readout_font.metrics("linespace") + 8)
        self.readout_frame.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.readout_frame.grid_propagate(False)
        self.readout_frame.grid_columnconfigure(1, weight=1)
        self.readout_frame.grid_rowconfigure(0, weight=1)
        self.readout_frame.grid_rowconfigure(1, weight=1)
        self.az_tag_label = tk.Label(self.readout_frame, text="AZ", font=self.readout_tag_font, padx=6)
        self.az_tag_label.grid(row=0, column=0, sticky="w")
        self.az_label = tk.Label(self.readout_frame, text="", font=self.readout_font, anchor="e", padx=6)
        self.az_label.grid(row=0, column=1, sticky="ew")
        self.el_tag_label = tk.Label(self.readout_frame, text="EL", font=self.readout_tag_font, padx=6)
        self.el_tag_label.grid(row=1, column=0, sticky="w")
        self.el_label = tk.Label(self.readout_frame, text="", font=self.readout_font, anchor="e", padx=6)
        self.el_label.grid(row=1, column=1, sticky="ew")
        self.readout_frame.bind("<Configure>", self._fit_readout)

        cal = self._frame(parent)
        cal.grid(row=1, column=0, sticky="ew", pady=(0, 2))
        self.cal_status_label = tk.Label(cal, text="", font=("Arial", 10, "bold"))
        self.cal_status_label.pack(side=tk.LEFT)
        self._make_button(cal, "Set Calibrated Zero", self._set_zero).pack(side=tk.RIGHT)

        tol = self._frame(parent)
        tol.grid(row=2, column=0, sticky="ew", pady=(2, 6))
        self._label(tol, "Tolerance (arc-sec):", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.tolerance_field = self._make_number(tol, 5.0, 0.1, 10000, 0.5, 1, width=7,
                                                 on_commit=lambda v: self._update_tolerance_status())
        self.tolerance_field.pack(side=tk.LEFT, padx=(6, 0))

        row1 = self._frame(parent)
        row1.grid(row=3, column=0, sticky="ew", pady=2)
        for c in range(3):
            row1.grid_columnconfigure(c, weight=1, uniform="stats1")
        self.signal_box = self._stat_box(row1, "Signal (%)", 0)
        rate_box = self._frame(row1, highlightthickness=1)
        rate_box.grid(row=0, column=1, padx=3, sticky="nsew")
        self._label(rate_box, "Samples / s", font=("Arial", 9)).pack(pady=(2, 0))
        rate_dd = self._make_dropdown(rate_box, self.rate_var, SAMPLE_RATES, width=5, font=("Arial", 12, "bold"))
        rate_dd.pack(pady=(0, 4))
        units_box = self._frame(row1, highlightthickness=1)
        units_box.grid(row=0, column=2, padx=3, sticky="nsew")
        self._label(units_box, "Data Units", font=("Arial", 9)).pack(pady=(2, 0))
        units_dd = self._make_dropdown(units_box, self.units_var, DATA_UNITS, width=11, font=("Arial", 10, "bold"))
        units_dd.pack(pady=(0, 4))
        units_dd.bind("<<ComboboxSelected>>", lambda e: (self._refresh_readouts(), self._refresh_charts()))

        row2 = self._frame(parent)
        row2.grid(row=4, column=0, sticky="ew", pady=2)
        for c in range(2):
            row2.grid_columnconfigure(c, weight=1, uniform="stats2")
        self.az_noise_box = self._stat_box(row2, "AZ Noise (RMS)", 0)
        self.el_noise_box = self._stat_box(row2, "EL Noise (RMS)", 1)

        self.az_fig = Figure(figsize=(3.9, 2.2), dpi=90)
        self.az_ax = self.az_fig.add_axes([0.20, 0.16, 0.76, 0.78])
        self.az_canvas, w = self._make_canvas(self.az_fig, parent, 300, 110)
        w.grid(row=5, column=0, sticky="nsew", pady=(6, 2))
        (self.az_line,) = self.az_ax.plot([], [], linewidth=0.9)

        self.el_fig = Figure(figsize=(3.9, 2.2), dpi=90)
        self.el_ax = self.el_fig.add_axes([0.20, 0.16, 0.76, 0.78])
        self.el_canvas, w = self._make_canvas(self.el_fig, parent, 300, 110)
        w.grid(row=6, column=0, sticky="nsew", pady=(2, 2))
        (self.el_line,) = self.el_ax.plot([], [], linewidth=0.9)

        self._make_button(parent, "Clear Charts", self._clear_charts).grid(row=7, column=0, pady=(4, 0))

    def _stat_box(self, parent, title, col):
        box = self._frame(parent, highlightthickness=1)
        box.grid(row=0, column=col, padx=3, sticky="nsew")
        self._label(box, title, font=("Arial", 9)).pack(pady=(2, 0))
        val = self._label(box, "--", font=(self.mono, 16, "bold"))
        val.pack(pady=(0, 4))
        box.value_label = val
        return box

    # ---------------- RIGHT: X-Y plot, metrics, capture, controls ----------------

    def _build_right(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(0, weight=1)

        xy = self._frame(parent)
        xy.grid(row=0, column=0, sticky="nsew")
        xy.grid_columnconfigure(0, weight=1)
        xy.grid_rowconfigure(1, weight=1)
        self._label(xy, "X-Y Plot (angle)", font=("Arial", 12, "bold"), anchor="w").grid(row=0, column=0, sticky="ew")
        # fixed-pixel square axes (not constrained layout, which resized
        # the plot whenever the tick labels changed width)
        self.xy_fig = Figure(figsize=(4.4, 3.0), dpi=90)
        self.xy_ax = self.xy_fig.add_axes([0.15, 0.15, 0.8, 0.8])
        (self.xy_connector,) = self.xy_ax.plot([], [], linewidth=1.0, linestyle=(0, (3, 2)), alpha=0.8)
        (self.xy_source_point,) = self.xy_ax.plot([0], [0], "+", markersize=11, markeredgewidth=2)
        self.xy_source_label = self.xy_ax.annotate("Source", (0, 0), xytext=(8, 8), textcoords="offset points",
                                                   fontsize=8, fontweight="bold")
        (self.xy_point,) = self.xy_ax.plot([], [], "o", markersize=9)
        self.xy_return_label = self.xy_ax.annotate("Return", (0, 0), xytext=(8, -12), textcoords="offset points",
                                                   fontsize=8, fontweight="bold")
        self.xy_return_label.set_visible(False)
        self.xy_canvas, w = self._make_canvas(self.xy_fig, xy, 300, 150)
        w.grid(row=1, column=0, sticky="nsew")
        pin_square_axes(self.xy_fig, self.xy_ax, XY_MARGINS)
        self.xy_ax.xaxis.set_major_formatter(FuncFormatter(tick_label))
        self.xy_ax.yaxis.set_major_formatter(FuncFormatter(tick_label))

        metrics = self._labelframe(parent, "Digital Metrology Metrics")
        metrics.grid(row=1, column=0, sticky="ew", pady=(6, 3))
        metrics.grid_columnconfigure(1, weight=1)
        self.metric_labels = {}
        rows = [
            ("centroid_px", "Centroid (sensor px)"),
            ("bias_px", "Calibrated Zero (sensor px)"),
            ("plate_scale", "Plate Scale (arc-sec/px)"),
            ("spot_size", "Spot Size (px) — smaller = sharper"),
            ("repeatability", "Repeatability, 2σ"),
            ("peak_signal", "Peak / Background Counts"),
            ("frame_rate", "Frame Rate Received (fps)"),
            ("exposure_time", "Exposure, Actual (ms)"),
            ("temperature", "Sensor Temperature (°C)"),
        ]
        for i, (key, text) in enumerate(rows):
            self._label(metrics, text, font=("Arial", 9), anchor="w").grid(row=i, column=0, sticky="w", padx=4)
            val = self._label(metrics, "--", role="readout", relief=tk.SUNKEN, width=17, font=(self.mono, 9, "bold"))
            val.grid(row=i, column=1, sticky="e", padx=4, pady=1)
            self.metric_labels[key] = val

        cap = self._labelframe(parent, "Capture")
        cap.grid(row=2, column=0, sticky="ew", pady=3)
        cap.grid_columnconfigure(0, weight=1)
        path_row = self._frame(cap)
        path_row.grid(row=0, column=0, sticky="ew", padx=4, pady=(2, 2))
        self.capture_dir_entry = tk.Entry(path_row, textvariable=self.capture_dir_var, font=("Arial", 9),
                                          relief=tk.FLAT, bd=2, highlightthickness=1)
        self.capture_dir_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.reg["entries"].append(self.capture_dir_entry)
        self._make_button(path_row, "Browse…", self._browse_capture_dir).pack(side=tk.LEFT, padx=(4, 0))

        btn_row = self._frame(cap)
        btn_row.grid(row=1, column=0, sticky="ew", padx=4, pady=2)
        for c in range(3):
            btn_row.grid_columnconfigure(c, weight=1, uniform="capbtn")
        self._make_button(btn_row, "Screenshot", self._take_screenshot).grid(row=0, column=0, sticky="ew", padx=2)
        self.record_btn = self._make_button(btn_row, "Record Video",
                                            self._toggle_recording)
        self.record_btn.grid(row=0, column=1, sticky="ew", padx=2)
        self._make_button(btn_row, "Save Raw Frame", self._save_raw_frame).grid(row=0, column=2, sticky="ew", padx=2)

        log_row = self._frame(cap)
        log_row.grid(row=2, column=0, sticky="ew", padx=4, pady=(2, 4))
        self._make_chip(log_row, "Log to CSV", self.log_var, kind="check", command=self._toggle_csv_log).pack(
            side=tk.LEFT)
        self.recording_indicator = tk.Canvas(log_row, width=14, height=14, highlightthickness=0)
        self.recording_indicator.pack(side=tk.LEFT, padx=(8, 4))
        self.reg["canvases"].append(self.recording_indicator)
        self.recording_dot = self.recording_indicator.create_oval(2, 2, 12, 12, fill="gray", outline="")
        self.capture_status_label = self._label(log_row, "", textvariable=self.capture_status_var,
                                                font=("Arial", 9), anchor="w", width=1)
        self.capture_status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # grid (not pack): the port column gives up width first, so the
        # Restart / Pause / Exit buttons always keep their full size
        ctrl = self._frame(parent)
        ctrl.grid(row=3, column=0, sticky="ew", pady=(6, 3))
        ctrl.grid_columnconfigure(0, weight=1)
        port_box = self._frame(ctrl)
        port_box.grid(row=0, column=0, sticky="w")
        # Reserved for a future serial link to the instrument electronics
        # (e.g. computer-controlled LEDs). Nothing uses it yet.
        self._label(port_box, "Instrument Port", font=("Arial", 8), anchor="w").pack(anchor="w")
        self._make_dropdown(port_box, self.com_var, self.serial_ports, width=14, font=("Arial", 9)).pack(anchor="w")
        self._make_button(ctrl, "Restart", self._restart, width=7).grid(row=0, column=1, padx=(6, 0))
        self.pause_btn = self._make_button(ctrl, "Pause", self._toggle_pause, font=("Arial", 11, "bold"), width=8)
        self.pause_btn.grid(row=0, column=2, padx=(6, 0))
        self.exit_btn = self._make_button(ctrl, "Exit", self._exit_app, width=7)
        self.exit_btn.grid(row=0, column=3, padx=(6, 0))

        # instrument info: wraps, but always INFO_LINES tall so a longer
        # text (mode/sensor name) can't change the X-Y plot's height
        self.info_label = self._label(parent, "", role="readout", relief=tk.SUNKEN, font=("Arial", 9),
                                      anchor="nw", justify="left", padx=4, pady=2, width=1,
                                      height=INFO_LINES)
        self.info_label.grid(row=4, column=0, sticky="ew", pady=(3, 0))
        self.info_label.bind("<Configure>", lambda e: self.info_label.configure(wraplength=max(100, e.width - 10)))


    # ================================================================
    # Tabs, modes, feed display
    # ================================================================

    def _show_tab(self, name):
        self.tab_var.set(name)
        self.pages[name].tkraise()

    def _set_mode(self, mode):
        """Both modes read the same camera; only the top-left display
        changes. Calibration and history are kept."""
        if mode not in MODES:
            return
        self.mode = mode
        self.mode_var.set(mode)
        self._configure_feed_axes()
        self._update_info_row()
        self._refresh_feed()

    def _init_feed_artists(self):
        """Create every feed artist ONCE; each frame only updates them.
        (Clearing and rebuilding the axes every frame was slow.)"""
        ax = self.cam_ax
        self.feed_img = ax.imshow(np.zeros((2, 2), dtype=np.uint8), cmap="gray", vmin=0, vmax=255,
                                  extent=[0, 1, 1, 0], interpolation="nearest", visible=False, zorder=0)
        self.feed_roi_rect = Rectangle((0, 0), 1, 1, fill=False, linestyle=(0, (4, 3)), linewidth=1.0,
                                       visible=False, zorder=1)
        ax.add_patch(self.feed_roi_rect)
        self.feed_hline = ax.axhline(0, linewidth=0.7, zorder=2)
        self.feed_vline = ax.axvline(0, linewidth=0.7, zorder=2)
        (self.feed_src,) = ax.plot([], [], "+", markersize=11, markeredgewidth=2, zorder=3)
        self.feed_src_text = ax.annotate("Source", (0, 0), xytext=(6, -11), textcoords="offset points",
                                         fontsize=8, fontweight="bold", zorder=3)
        (self.feed_conn,) = ax.plot([], [], linewidth=1.0, linestyle=(0, (3, 2)), alpha=0.85, zorder=3)
        (self.feed_ret,) = ax.plot([], [], "o", markerfacecolor="none", markersize=14, markeredgewidth=1.5,
                                   zorder=4)
        self.feed_ret_text = ax.annotate("Return", (0, 0), xytext=(10, 10), textcoords="offset points",
                                         fontsize=8, fontweight="bold", zorder=4)
        self.feed_fps_text = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=10,
                                     fontfamily="monospace", fontweight="bold", va="top", ha="left", zorder=5)
        self.feed_dims_text = ax.text(0.98, 0.97, "", transform=ax.transAxes, fontsize=8,
                                      fontfamily="monospace", va="top", ha="right", zorder=5)
        self.feed_msg_text = ax.text(0.5, 0.5, "", transform=ax.transAxes, ha="center", va="center",
                                     fontsize=10, zorder=5)
        self.feed_sat_text = ax.text(0.5, 0.03, "SATURATED", transform=ax.transAxes, ha="center",
                                     va="bottom", fontsize=10, fontweight="bold", visible=False, zorder=5)
        # The axes box is a fixed-pixel square (pin_square_axes) and the
        # limits are always a square region, so the aspect is 1:1 without
        # letting matplotlib resize the box.
        ax.set_aspect("auto")
        ax.xaxis.set_major_formatter(FuncFormatter(tick_label))
        ax.yaxis.set_major_formatter(FuncFormatter(tick_label))

    def _configure_feed_axes(self):
        """Both modes use the SAME plot box, ticks and axis labels, so
        switching modes changes only what's drawn inside the box. The
        grid is hidden over the live image so it doesn't hide the spot."""
        ax = self.cam_ax
        t = self.theme
        metrology = self.mode == "Metrology"
        self.feed_title_label.config(text="Live Camera Image (ROI)" if metrology
                                     else "Source / Return Position (whole sensor)")
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True,
                       colors=t["plot_fg"], labelsize=7)
        if metrology:
            ax.grid(False)
        else:
            ax.grid(True, color=t["grid"], linewidth=0.5)
        ax.set_xlabel("sensor x (px)", color=t["plot_fg"], fontsize=8)
        ax.set_ylabel("sensor y (px)", color=t["plot_fg"], fontsize=8)

    def _feed_view(self):
        """(center_x, center_y, half_size) of the square region shown:
        the ROI in Metrology, the whole sensor in Alignment. A
        non-square region is padded to a square around its center."""
        if self.mode == "Metrology" and self.source is not None:
            ox, oy = self.display_offset
            cx, cy = ox + self.frame_w / 2.0, oy + self.frame_h / 2.0
            half = max(self.frame_w, self.frame_h) / 2.0
        else:
            cx, cy = self.sensor_w / 2.0, self.sensor_h / 2.0
            half = max(self.sensor_w, self.sensor_h) / 2.0
        return cx, cy, half

    def _feed_limits(self):
        cx, cy, half = self._feed_view()
        return (cx - half, cx + half), (cy + half, cy - half)    # y down, like the image

    def _refresh_feed(self):
        t = self.theme
        ax = self.cam_ax
        metrology = self.mode == "Metrology"

        if metrology and self.last_display_frame is not None:
            ox, oy = self.display_offset
            dh, dw = self.last_display_frame.shape[:2]
            s = self.display_step
            self.feed_img.set_data(self.last_display_frame)
            self.feed_img.set_extent([ox, ox + dw * s, oy + dh * s, oy])
            self.feed_img.set_visible(True)
        else:
            self.feed_img.set_visible(False)

        # ROI outline on the whole-sensor graph
        ox, oy = self.display_offset
        roi_smaller = self.source is not None and (self.frame_w < self.sensor_w or self.frame_h < self.sensor_h)
        self.feed_roi_rect.set_bounds(ox, oy, self.frame_w, self.frame_h)
        self.feed_roi_rect.set_visible(roi_smaller and not metrology)

        bx, by = self.bias_x, self.bias_y
        self.feed_hline.set_ydata([by, by])
        self.feed_vline.set_xdata([bx, bx])
        self.feed_src.set_data([bx], [by])
        self.feed_src_text.xy = (bx, by)

        has_ret = self.last_cx is not None
        if has_ret:
            self.feed_ret.set_data([self.last_cx], [self.last_cy])
            self.feed_ret_text.xy = (self.last_cx, self.last_cy)
            self.feed_conn.set_data([bx, self.last_cx], [by, self.last_cy])
        for a in (self.feed_ret, self.feed_ret_text, self.feed_conn):
            a.set_visible(has_ret)

        if self.source is None:
            self.feed_msg_text.set_text("No camera connected\nChoose a source on the Camera tab")
            self.feed_msg_text.set_color(t["plot_fg"])
            self.feed_msg_text.set_position((0.5, 0.75))     # clear of the centered Source marker
        elif not has_ret and self.source_frame_id >= 0:
            self.feed_msg_text.set_text("No return detected")
            self.feed_msg_text.set_color(t["bad"])
            self.feed_msg_text.set_position((0.5, 0.12))
        else:
            self.feed_msg_text.set_text("")
        self.feed_fps_text.set_text(f"{self.measured_fps:.1f} fps" if self.measured_fps > 0 else "-- fps")
        self.feed_dims_text.set_text(f"{self.frame_w}×{self.frame_h} px" if self.source is not None else "")
        self.feed_sat_text.set_visible(self.last_saturated)

        xlim, ylim = self._feed_limits()
        if (xlim, ylim) != getattr(self, "_feed_lims_applied", None):
            # grid lines symmetric about the view center (= sensor center
            # in Alignment), so the default Source crosshair sits on the
            # center grid line
            cx, cy, half = self._feed_view()
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
            ax.xaxis.set_major_locator(FixedLocator(centered_ticks(cx - half, cx + half, cx)))
            ax.yaxis.set_major_locator(FixedLocator(centered_ticks(cy - half, cy + half, cy)))
            self._feed_lims_applied = (xlim, ylim)
        self.cam_canvas.draw_idle()

    def _apply_feed_colors(self):
        t = self.theme
        self.cam_fig.set_facecolor(t["plot_bg"])
        self.cam_ax.set_facecolor(t["plot_bg"])
        for spine in self.cam_ax.spines.values():
            spine.set_color(t["plot_fg"])
        for a in (self.feed_hline, self.feed_vline):
            a.set_color(t["bad"])
        self.feed_src.set_color(t["bad"])
        self.feed_src_text.set_color(t["bad"])
        self.feed_ret.set_markeredgecolor(t["ok"])
        self.feed_ret_text.set_color(t["ok"])
        self.feed_conn.set_color(t["marker"])
        self.feed_roi_rect.set_edgecolor(t["marker"])
        self.feed_fps_text.set_color(t["marker"])
        self.feed_dims_text.set_color(t["marker"])
        self.feed_sat_text.set_color(t["bad"])
        self._configure_feed_axes()

    # ================================================================
    # Camera source
    # ================================================================

    def _update_info_row(self):
        if self.source is None:
            sensor = "--"
        elif self.source.is_simulator:
            sensor = "Simulator"
        elif self.source.is_emulated:
            sensor = "pylon Emulator"
        else:
            sensor = "a2A4504 ace2"
        self.info_label.config(text=f"Mode: {self.mode}  ·  Sensor: {sensor}  ·  240-270 nm  ·  "
                                    f"Broadband filter  ·  Focus 30 cm - ∞  ·  Software {SOFTWARE_REV}")

    def _update_source_controls(self):
        """Enable only the controls that apply to the selected source.
        Nothing is hidden or re-packed, so nothing moves."""
        sim = self.source_var.get() == "Simulator"
        connected = self.source is not None
        for chip in self.format_chips:
            (self._disabled_chips.add if sim else self._disabled_chips.discard)(chip)
        self.roi_combo.set_enabled(not sim)
        self.gain_ctl.set_enabled(True)
        self.focal_field.set_enabled(not sim)
        test_chip_off = not sim
        (self._disabled_chips.add if test_chip_off else self._disabled_chips.discard)(self.test_mode_chip)
        self.test_az_field.set_enabled(sim)
        self.test_el_field.set_enabled(sim)
        if sim:
            self.test_note.config(text="Places the simulated spot at an exact AZ/EL so you can check the "
                                       "readout math against a value you choose.")
        else:
            self.test_note.config(text="Test Point works with the Simulator source only. Choose "
                                       "“Simulator” on the Camera tab to use it.")
            if self.test_mode_var.get():
                self.test_mode_var.set(False)
        self.auto_exp_btn.configure(fg=self.theme["fg"] if connected else self.theme["neutral"])
        self._update_plate_scale_label()
        self._refresh_toggle_colors()

    def _camera_settings(self):
        roi = roi_size_from_choice(self.roi_var.get())
        return {"pixel_format": self.pixel_format_var.get(), "roi_size": roi,
                "roi_center": (self.bias_x, self.bias_y), "flip_x": self.flip_x_var.get(),
                "flip_y": self.flip_y_var.get()}

    def _select_source(self, name, force=False, interactive=True):
        if name not in SOURCES:
            return
        if name == self.source_var.get() and self.source is not None and not force:
            return
        if self._recording:
            self._stop_recording()
        self.source_var.set(name)
        self._connect_source(interactive)
        self._update_source_controls()
        self._reset_measurement()

    def _connect_source(self, interactive=True):
        self._disconnect_source(reset=False)
        name = self.source_var.get()
        src = SimulatedSource() if name == "Simulator" else BaslerSource(use_emulator=(name == "Emulator"))
        try:
            src.connect(self._camera_settings())
        except ImportError as exc:
            src.disconnect()
            fix = f'"{sys.executable}" -m pip install pypylon'
            self._set_camera_status("pypylon is not installed for this Python.\n"
                                    "Fix: run the command shown in the terminal, then Reconnect.")
            print(f"[pypylon] import failed: {exc}\n[pypylon] this app runs: {sys.executable}\n"
                  f"[pypylon] install it into this Python with:\n    {fix}")
            if interactive:
                messagebox.showerror(
                    "pypylon not found",
                    f"pypylon could not be imported:\n{exc}\n\nThis app is running Python at:\n"
                    f"{sys.executable}\n\nInstall pypylon into THAT Python by running this in a "
                    f"terminal (also printed in the terminal for copy/paste):\n\n{fix}")
            return False
        except Exception as exc:
            src.disconnect()
            self._set_camera_status(f"Not connected: {exc}")
            return False

        self.source = src
        self.source_frame_id = -1
        self.sensor_w, self.sensor_h = src.sensor_w, src.sensor_h
        self.peak_max = src.max_value

        # re-range the setting controls to THIS camera's real limits, then
        # push the current values and show what the camera accepted
        lim = src.limits
        self.exposure_ctl.set_range(*lim["exposure_ms"])
        self.gain_ctl.set_range(*lim["gain_db"])
        self.black_ctl.set_range(*lim["black_level"])
        self.fps_ctl.set_range(*lim["frame_rate"])
        for ctl, setter in ((self.exposure_ctl, src.set_exposure_ms), (self.gain_ctl, src.set_gain_db),
                            (self.black_ctl, src.set_black_level), (self.fps_ctl, src.set_frame_rate)):
            applied = setter(ctl.get())
            if applied is not None:
                ctl.set(applied)
        self.pixel_format_var.set("Mono12" if src.max_value > 255 else "Mono8")
        self.flip_x_var.set(src.flip_x)
        self.flip_y_var.set(src.flip_y)
        src.threshold_ratio = self._threshold_ratio()
        try:
            src.start()
        except Exception as exc:
            src.disconnect()
            self.source = None
            self._set_camera_status(f"Could not start acquisition: {exc}")
            return False

        if src.is_simulator:
            self._set_camera_status(f"Simulator — {src.width}×{src.height} px, "
                                    f"{ARCSEC_PER_PIXEL_SIM:g} arc-sec/px (no hardware)")
        else:
            tag = "  [EMULATOR]" if src.is_emulated else ""
            self._set_camera_status(f"Connected: {src.model_name}  SN {src.serial}{tag}\n"
                                    f"{src.sensor_w}×{src.sensor_h} sensor, {src.pixel_format}")
            if src.warnings:
                self._note("⚠ " + "; ".join(src.warnings), 30.0)
                print("[camera] " + "\n[camera] ".join(src.warnings))
        self._poll_camera_info(force=True)
        return True

    def _disconnect_source(self, reset=True):
        if self._auto_exp_after_id is not None:
            self.root.after_cancel(self._auto_exp_after_id)
            self._auto_exp_after_id = None
        if self.source is not None:
            try:
                self.source.disconnect()
            except Exception:
                pass
            self.source = None
            self._set_camera_status("Not connected")
            self.cam_info_var.set("")
        if reset:
            self._update_source_controls()
            self._reset_measurement()

    def _on_disconnect_clicked(self):
        self._disconnect_source(reset=True)

    def _set_camera_status(self, text):
        self._camera_status_base = text
        self._status_note = None
        self._show_status(text)

    def _note(self, text, seconds=8.0):
        """Show a message on the status box's last line for a few seconds
        (frame updates no longer erase it immediately)."""
        self._status_note = (text, time.perf_counter() + seconds)
        self._compose_status()

    def _compose_status(self):
        text = self._camera_status_base
        note = self._status_note
        if note is not None and time.perf_counter() < note[1]:
            text += "\n" + note[0]
        elif self.last_saturated and self.source is not None:
            text += "\n⚠ SATURATED — lower exposure or gain"
        self._show_status(text)

    def _show_status(self, text):
        """Fixed-height status box: extra lines are dropped (and printed
        to the terminal) instead of resizing anything."""
        lines = text.split("\n")
        if len(lines) > STATUS_LINES:
            print("[camera status]", text)
            lines = lines[:STATUS_LINES - 1] + [lines[-1]]
        new = "\n".join(lines)
        if self.camera_status_var.get() != new:
            self.camera_status_var.set(new)

    def _poll_camera_info(self, force=False):
        """Sensor temperature, achievable frame rate, actual exposure —
        read every few seconds (camera reads aren't free)."""
        now = time.perf_counter()
        if self.source is None or (not force and now - self._last_poll < TEMP_POLL_S):
            return
        self._last_poll = now
        src = self.source
        self.temp_c = src.temperature_c()
        self.camera_max_fps = src.resulting_fps()
        self.exposure_actual_ms = src.get_exposure_ms()
        parts = []
        if self.temp_c is not None:
            parts.append(f"Sensor {self.temp_c:.1f} °C")
        if self.camera_max_fps is not None:
            parts.append(f"camera delivering ≤ {self.camera_max_fps:.1f} fps")
        roi = "full sensor" if self.frame_w >= self.sensor_w else f"ROI {self.frame_w}×{self.frame_h}"
        parts.append(roi)
        self.cam_info_var.set("  ·  ".join(parts))

    # ---------------- camera settings ----------------

    def _on_exposure_change(self, ms):
        if self.source is None:
            return ms
        applied = self.source.set_exposure_ms(ms)
        self.exposure_actual_ms = applied
        return applied

    def _on_gain_change(self, db):
        if self.source is None:
            return db
        applied = self.source.set_gain_db(db)
        if applied is None and not self.source.is_simulator:
            self._note("Gain is not adjustable on this camera")
        return applied

    def _on_black_change(self, value):
        return value if self.source is None else self.source.set_black_level(value)

    def _on_fps_change(self, fps):
        if self.source is None:
            return fps
        applied = self.source.set_frame_rate(fps)
        self._poll_camera_info(force=True)
        return applied

    def _auto_exposure(self):
        src = self.source
        if src is None:
            self._note("Connect a camera first")
            return
        if not src.auto_exposure_once():
            self._note("Auto exposure is not supported by this camera")
            return
        self._note("Auto exposure running…", 6.0)
        deadline = time.perf_counter() + 4.0

        def _poll():
            self._auto_exp_after_id = None
            if self.source is not src:
                return
            if src.auto_exposure_busy() and time.perf_counter() < deadline:
                self._auto_exp_after_id = self.root.after(200, _poll)
                return
            src.auto_exposure_stop()          # always leave the camera in MANUAL exposure
            ms = src.get_exposure_ms()
            if ms is not None:
                self.exposure_ctl.set(ms)
                self.exposure_actual_ms = ms
                self._note(f"Auto exposure set {ms:.3f} ms (now manual)")
        _poll()

    def _select_pixel_format(self, pf):
        if pf == self.pixel_format_var.get() or self._using_simulator():
            return
        self.pixel_format_var.set(pf)
        self._apply_geometry_change(pixel_format=pf)

    def _toggle_flip(self, var):
        var.set(not var.get())
        self._apply_geometry_change(flip=True)

    def _apply_geometry_change(self, pixel_format=None, roi=False, flip=False):
        """Bit depth / ROI / flip: the camera only accepts these while
        stopped, so grabbing pauses for a moment. Calibration is kept
        unless the image orientation changed (flip)."""
        src = self.source
        if src is None:
            return
        kwargs = {}
        if pixel_format:
            kwargs["pixel_format"] = pixel_format
        if roi:
            kwargs["roi_size"] = roi_size_from_choice(self.roi_var.get())
            kwargs["roi_center"] = (self.bias_x, self.bias_y)
        if flip:
            kwargs["flip_x"] = self.flip_x_var.get()
            kwargs["flip_y"] = self.flip_y_var.get()
        if not src.is_simulator and "roi_size" not in kwargs:
            # the camera re-applies the ROI on every reconfigure, so always
            # pass the current one (otherwise a bit-depth change would
            # silently reset the ROI to full sensor)
            kwargs["roi_size"] = roi_size_from_choice(self.roi_var.get())
            kwargs["roi_center"] = (self.bias_x, self.bias_y)
        err = src.reconfigure(**kwargs)
        if err:
            self._note(f"Setting rejected: {err}")
        # reflect what the camera actually did
        self.flip_x_var.set(src.flip_x)
        self.flip_y_var.set(src.flip_y)
        if not src.is_simulator:
            self.pixel_format_var.set("Mono12" if src.max_value > 255 else "Mono8")
        self.peak_max = src.max_value
        self.frame_w, self.frame_h = src.width, src.height
        self.display_offset = src.offset
        self.source_frame_id = -1
        self.last_display_frame = None
        if self.calibrated and self._cal_key != self._current_cal_key():
            self._clear_calibration(note="Image orientation changed — calibration cleared. "
                                         "Set Calibrated Zero again.")
        self._poll_camera_info(force=True)
        self._refresh_feed()

    # ================================================================
    # Angle conversion and calibration
    # ================================================================

    def _using_simulator(self):
        return self.source is not None and self.source.is_simulator

    def _metrology_arcsec_per_px(self):
        f_mm = max(1.0, self.focal_field.get())
        return (CAMERA_PIXEL_UM * 1e-3) / (2.0 * f_mm) * ARC_SEC_PER_RAD

    def _arcsec_per_px(self):
        if self.source_var.get() == "Simulator":
            return ARCSEC_PER_PIXEL_SIM
        return self._metrology_arcsec_per_px()

    def _update_plate_scale_label(self):
        self.plate_scale_label.config(text=f"= {self._arcsec_per_px():.4f} arc-sec/px")
        self._refresh_readouts()

    def _current_cal_key(self):
        src = self.source
        if src is None:
            return None
        # the camera serial makes a saved calibration belong to ONE unit
        return [self.source_var.get(), str(src.serial), int(src.sensor_w), int(src.sensor_h),
                bool(src.flip_x), bool(src.flip_y)]

    def _set_zero(self):
        if self.last_cx is None:
            messagebox.showwarning("Calibration", "No return spot detected — cannot set zero.")
            return
        self.bias_x, self.bias_y = self.last_cx, self.last_cy
        self.calibrated = True
        self._cal_key = self._current_cal_key()
        self._update_cal_label()
        if self._using_simulator() and self.test_mode_var.get():
            # keep the simulated spot where it is: move the Test Point
            # origin to the new zero, then zero the target fields
            self.test_point_origin_x, self.test_point_origin_y = self.bias_x, self.bias_y
            self.test_az_field.set(0)
            self.test_el_field.set(0)
        self.last_az, self.last_el = 0.0, 0.0
        self._refresh_readouts()
        self._refresh_feed()

    def _clear_calibration(self, note=None):
        self.bias_x, self.bias_y = self.sensor_w / 2.0, self.sensor_h / 2.0
        self.test_point_origin_x = self.test_point_origin_y = FRAME_SIZE / 2.0
        self.calibrated = False
        self._cal_key = None
        self._update_cal_label()
        if note:
            self._note(note, 10.0)

    def _update_cal_label(self):
        t = self.theme
        self.cal_status_label.configure(bg=t["panel_bg"], fg=t["ok"] if self.calibrated else t["bad"],
                                        text="● CALIBRATED" if self.calibrated else "● UNCALIBRATED")

    def _reset_measurement(self):
        """After a source change: new sensor geometry, clear history.
        Calibration is kept only if it belongs to this same sensor and
        orientation (e.g. after Reconnect)."""
        src = self.source
        if src is not None:
            self.sensor_w, self.sensor_h = src.sensor_w, src.sensor_h
            self.frame_w, self.frame_h = src.width, src.height
            self.display_offset = src.offset
            self.peak_max = src.max_value
        else:
            # nothing connected: show the selected source's nominal sensor
            # (not whatever the previous source happened to be)
            size = FRAME_SIZE if self.source_var.get() == "Simulator" else CAMERA_WIDTH_PX
            self.sensor_w = self.sensor_h = self.frame_w = self.frame_h = size
            self.display_offset = (0, 0)
        if not self.calibrated:
            self._clear_calibration()                  # re-center on this sensor
        elif src is not None and self._cal_key != self._current_cal_key():
            # a saved calibration only applies to the same camera unit and
            # orientation it was made on. With nothing connected it's kept
            # (so opening the app without the camera doesn't lose it).
            self._clear_calibration(note="Saved calibration was made on a different camera or "
                                         "orientation — cleared. Set Calibrated Zero again.")
        if self._using_simulator():
            self.source.reset_walk()
        self.last_display_frame = None
        self.display_step = 1
        self.last_cx = self.last_cy = None
        self.last_az = self.last_el = None
        self.last_spot_px = None
        self.last_signal = 0
        self.last_peak_raw = 0
        self.last_background = 0.0
        self.last_saturated = False
        self.in_tolerance = None
        self._frame_timestamps.clear()
        self.measured_fps = 0.0
        self._clear_charts()
        self._update_info_row()
        self._refresh_readouts()
        self._refresh_tolerance_indicator()
        self._refresh_feed()

    # ================================================================
    # Acquisition loop
    # ================================================================

    def _interval_ms(self):
        rate = max(0.5, self._safe_float(self.rate_var, 10.0))
        return max(30, int(1000 / rate))

    def _update_loop(self):
        if not self.running:
            return
        if not self.paused:
            try:
                self._acquire_tick()
            except Exception as exc:          # never let one bad frame kill the loop
                print(f"[loop] {type(exc).__name__}: {exc}")
        if time.perf_counter() - self._last_autosave > AUTOSAVE_S:
            self._save_session()
        self._loop_after_id = self.root.after(self._interval_ms(), self._update_loop)

    def _threshold_ratio(self):
        return max(0.01, min(0.99, self.threshold_field.get() / 100.0))

    def _acquire_tick(self):
        src = self.source
        if src is None:
            return
        if not src.is_alive():
            self._disconnect_source(reset=False)
            self._set_camera_status("Camera disconnected.\nCheck the USB cable, then press Reconnect.")
            self._update_source_controls()
            self._refresh_feed()
            return
        src.threshold_ratio = self._threshold_ratio()
        if src.is_simulator:
            if self.test_mode_var.get():
                src.forced_pixel = (self.test_point_origin_x + self.test_az_field.get() / ARCSEC_PER_PIXEL_SIM,
                                    self.test_point_origin_y - self.test_el_field.get() / ARCSEC_PER_PIXEL_SIM)
            else:
                src.forced_pixel = None

        frame_id, res = src.get_latest()
        if res is None or frame_id == self.source_frame_id:
            if src.last_error:
                self._note(src.last_error, 3.0)
            return
        self.source_frame_id = frame_id

        self.last_display_frame = res["display"]
        self.display_step = res["step"]
        self.display_offset = res["offset"]
        self.frame_w, self.frame_h = res["width"], res["height"]
        self.last_saturated = res["saturated"]
        self.last_spot_px = res["spot_px"]
        self.last_background = res["background"]
        self.last_peak_raw = int(round(res["peak"]))
        self.peak_max = src.max_value
        self._record_frame_time(res["t"])
        self._poll_camera_info()

        self._compose_status()

        cx, cy = res["cx"], res["cy"]
        self.last_cx, self.last_cy = cx, cy
        if cx is not None:
            aspp = self._arcsec_per_px()
            self.last_az = (cx - self.bias_x) * aspp
            self.last_el = -(cy - self.bias_y) * aspp
            self.last_signal = int(round(min(100.0, 100.0 * res["peak"] / float(src.max_value))))
        else:
            # lost the spot: show NO SIGNAL rather than a stale reading
            self.last_az = self.last_el = None
            self.last_signal = 0

        self._n += 1
        self.sample_index.append(self._n)
        self.az_hist.append(self.last_az if self.last_az is not None else float("nan"))
        self.el_hist.append(self.last_el if self.last_el is not None else float("nan"))

        self._write_csv_row()
        self._refresh_feed()
        self._refresh_readouts()
        self._refresh_charts()
        self._update_tolerance_status()

        if self._recording and self._recording_mode == "frames":
            frame_img, frame_err = self._capture_window_image()
            self._recording_frames.append(frame_img)
            if frame_err is not None and self._recording_capture_error is None:
                self._recording_capture_error = frame_err

    def _record_frame_time(self, t):
        self._frame_timestamps.append(t)
        if len(self._frame_timestamps) >= 2:
            span = self._frame_timestamps[-1] - self._frame_timestamps[0]
            if span > 0:
                self.measured_fps = (len(self._frame_timestamps) - 1) / span

    # ================================================================
    # Readouts and charts
    # ================================================================

    def _pixel_mm(self):
        """Sensor pixel pitch in mm (2.74 µm on the a2A4504)."""
        return CAMERA_PIXEL_UM / 1000.0

    def _arcsec_to_mm(self, value_arcsec):
        """Angle reading -> linear spot displacement on the sensor, in mm.
        Same signs as AZ (right +) and EL (up +)."""
        aspp = self._arcsec_per_px()
        if value_arcsec is None or not aspp:
            return None
        return value_arcsec / aspp * self._pixel_mm()

    def _convert(self, value_arcsec):
        if value_arcsec is None:
            return None
        unit = self.units_var.get()
        if unit == "Micro-Radian":
            return value_arcsec * MICRO_RAD_PER_ARC_SEC
        if unit == "Millimeter":
            return self._arcsec_to_mm(value_arcsec)
        return value_arcsec

    def _units_short(self):
        return {"Micro-Radian": "µrad", "Millimeter": "mm"}.get(self.units_var.get(), "arc-sec")

    def _decimals(self):
        """mm values are small (1 px = 0.00274 mm), so show more digits."""
        return 4 if self.units_var.get() == "Millimeter" else 2

    def _fit_readout(self, event=None):
        """Largest digit size that fits the column, so the AZ/EL numbers
        are never cut off (window size and available fonts vary).
        Text width scales linearly with font size, so measure once at a
        reference size and scale."""
        width = self.readout_frame.winfo_width()
        if width < 80:
            return
        ref = 40
        digits_w = tkfont.Font(family=self.mono, size=ref, weight="bold").measure("+00000.00")
        tag_w = tkfont.Font(family="Arial", size=int(ref * 0.8), weight="bold").measure("AZ")
        padding = 4 * 6 + 12            # label padx on both labels + margin
        size = int(ref * (width - padding) / float(digits_w + tag_w))
        size = max(12, min(48, size))
        if self.readout_font.cget("size") != size:
            self.readout_font.configure(size=size)
            self.readout_tag_font.configure(size=max(10, int(size * 0.8)))
        # fixed height from the font, and propagate off: the digits can
        # then never widen the column (which made the size oscillate)
        height = 2 * self.readout_font.metrics("linespace") + 8
        if abs(self.readout_frame.winfo_height() - height) > 1:
            self.readout_frame.configure(height=height)

    def _stats(self, values):
        vals = np.array([v for v in list(values)[-STATS_WINDOW:] if v == v], dtype=float)  # drop NaN
        if vals.size < 3:
            return None
        return float(vals.std())

    def _refresh_readouts(self):
        if not hasattr(self, "az_label"):
            return
        az, el = self._convert(self.last_az), self._convert(self.last_el)
        d = self._decimals()
        self.az_label.config(text=f"{az:+08.{d}f}" if az is not None else "  NO SIG")
        self.el_label.config(text=f"{el:+08.{d}f}" if el is not None else "  NO SIG")
        self.signal_box.value_label.config(text=str(self.last_signal))
        u = self._units_short()
        az_rms, el_rms = self._stats(self.az_hist), self._stats(self.el_hist)
        if az_rms is not None and el_rms is not None:
            nd = self._decimals() + 1
            self.az_noise_box.value_label.config(text=f"{self._convert(az_rms):.{nd}f}")
            self.el_noise_box.value_label.config(text=f"{self._convert(el_rms):.{nd}f}")
            self.metric_labels["repeatability"].config(text=f"{self._convert(2 * max(az_rms, el_rms)):.{nd}f} {u}")
        else:
            self.az_noise_box.value_label.config(text="--")
            self.el_noise_box.value_label.config(text="--")
            self.metric_labels["repeatability"].config(text="--")
        m = self.metric_labels
        m["centroid_px"].config(text=f"{self.last_cx:.2f}, {self.last_cy:.2f}" if self.last_cx is not None else "--")
        m["bias_px"].config(text=f"{self.bias_x:.2f}, {self.bias_y:.2f}")
        m["plate_scale"].config(text=f"{self._arcsec_per_px():.4f}")
        m["spot_size"].config(text=f"{self.last_spot_px:.2f}" if self.last_spot_px is not None else "--")
        m["peak_signal"].config(text=f"{self.last_peak_raw} / {self.last_background:.0f}  of {self.peak_max}"
                                if self.source is not None else "--")
        m["frame_rate"].config(text=f"{self.measured_fps:.2f}" if self.measured_fps > 0 else "--")
        exp = self.exposure_actual_ms if self.exposure_actual_ms is not None else None
        m["exposure_time"].config(text=f"{exp:.3f}" if exp is not None else "--")
        m["temperature"].config(text=f"{self.temp_c:.1f}" if self.temp_c is not None else "--")

    def _refresh_charts(self):
        xs = list(self.sample_index)
        az = np.array([self._convert(v) if v == v else np.nan for v in self.az_hist], dtype=float)
        el = np.array([self._convert(v) if v == v else np.nan for v in self.el_hist], dtype=float)
        u = self._units_short()
        for ax, line, vals, canvas, name in ((self.az_ax, self.az_line, az, self.az_canvas, "Azimuth"),
                                             (self.el_ax, self.el_line, el, self.el_canvas, "Elevation")):
            line.set_data(xs, vals)
            if xs:
                ax.set_xlim(max(0, xs[0]), xs[-1] + 1)
            finite = vals[np.isfinite(vals)] if vals.size else vals
            if finite.size:
                lo, hi = float(finite.min()), float(finite.max())
                pad = max(0.001 if u == "mm" else 0.5, (hi - lo) * 0.2)
                ax.set_ylim(lo - pad, hi + pad)
            ax.set_ylabel(f"{name} ({u})", color=self.theme["plot_fg"], fontsize=8)
            canvas.draw_idle()

        last_az = self._convert(self.last_az)
        last_el = self._convert(self.last_el)
        if last_az is not None:
            self.xy_point.set_data([last_az], [last_el])
            self.xy_connector.set_data([0, last_az], [0, last_el])
            self.xy_return_label.xy = (last_az, last_el)
        visible = last_az is not None
        for a in (self.xy_point, self.xy_connector, self.xy_return_label):
            a.set_visible(visible)
        half = self._convert((max(self.sensor_w, self.sensor_h) / 2.0) * self._arcsec_per_px())
        if half != getattr(self, "_xy_half_applied", None):
            self.xy_ax.set_xlim(-half, half)
            self.xy_ax.set_ylim(-half, half)
            ticks = FixedLocator(centered_ticks(-half, half, 0.0, target=6))
            self.xy_ax.xaxis.set_major_locator(ticks)
            self.xy_ax.yaxis.set_major_locator(FixedLocator(centered_ticks(-half, half, 0.0, target=6)))
            self._xy_half_applied = half
        self.xy_ax.set_xlabel(f"Azimuth ({u})", color=self.theme["plot_fg"], fontsize=8)
        self.xy_ax.set_ylabel(f"Elevation ({u})", color=self.theme["plot_fg"], fontsize=8)
        self.xy_canvas.draw_idle()

    def _clear_charts(self):
        self.sample_index.clear()
        self.az_hist.clear()
        self.el_hist.clear()
        self._n = 0
        self._refresh_charts()
        self._refresh_readouts()

    # ================================================================
    # Tolerance alert
    # ================================================================

    def _update_tolerance_status(self):
        if self.last_az is None:
            self.in_tolerance = None
        else:
            tol = self.tolerance_field.get()
            now_ok = abs(self.last_az) <= tol and abs(self.last_el) <= tol
            if self.in_tolerance is not None and now_ok != self.in_tolerance:
                self._play_alert(now_ok)
            self.in_tolerance = now_ok
        self._refresh_tolerance_indicator()

    def _refresh_tolerance_indicator(self):
        if self.in_tolerance is None:
            color = NO_SIGNAL_COLOR
        else:
            color = TOLERANCE_OK_COLOR if self.in_tolerance else TOLERANCE_BAD_COLOR
        fg = contrast_text(color)
        self.readout_frame.configure(bg=color)
        for w in (self.az_tag_label, self.el_tag_label, self.az_label, self.el_label):
            w.configure(bg=color, fg=fg)

    def _ensure_alert_sounds(self):
        try:
            tmp_dir = tempfile.gettempdir()
            self._alert_wav_out = os.path.join(tmp_dir, "duv_alert_out_of_tolerance_v2.wav")
            self._alert_wav_in = os.path.join(tmp_dir, "duv_alert_in_tolerance_v2.wav")
            if not os.path.exists(self._alert_wav_out):
                self._write_chime(self._alert_wav_out, descending=True)
            if not os.path.exists(self._alert_wav_in):
                self._write_chime(self._alert_wav_in, descending=False)
        except Exception:
            self._alert_wav_out = self._alert_wav_in = None

    @staticmethod
    def _write_chime(path, descending=True):
        """Soft bell tone (a few decaying harmonics) — calm, not an alarm."""
        sr, dur = 44100, 1.6
        n = int(sr * dur)
        t_arr = np.linspace(0, dur, n, False)
        base = 293.66 if descending else 392.00
        data = np.zeros(n)
        for ratio, amp, decay in ((1.00, 1.00, 2.2), (2.00, 0.32, 3.0), (2.76, 0.16, 3.6), (4.00, 0.07, 4.2)):
            data += amp * np.sin(2 * np.pi * base * ratio * t_arr) * np.exp(-decay * t_arr)
        data /= max(1e-9, np.max(np.abs(data)))
        data *= 0.18
        attack = max(1, int(sr * 0.015))
        data[:attack] *= np.linspace(0, 1, attack)
        pcm = (np.clip(data, -1, 1) * 32767).astype(np.int16)
        with wave.open(path, "w") as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(sr)
            f.writeframes(pcm.tobytes())

    def _play_alert(self, went_in_tolerance):
        path = self._alert_wav_in if went_in_tolerance else self._alert_wav_out
        if not path or not os.path.exists(path):
            return
        try:
            system = platform.system()
            if system == "Darwin":
                subprocess.Popen(["afplay", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            elif system == "Windows":
                import winsound
                winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC)
            else:
                player = shutil.which("paplay") or shutil.which("aplay")
                if player:
                    subprocess.Popen([player, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    @staticmethod
    def _safe_float(var, default=0.0):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            return default

    # ================================================================
    # Theming
    # ================================================================

    def _sync_chip(self, widget, selected, base_text, kind):
        t = self.theme
        prefix = ("☑ " if selected else "☐ ") if kind == "check" else ("● " if selected else "○ ")
        if widget in self._disabled_chips:
            widget.configure(bg=t["panel_bg"], fg=t["neutral"], text=prefix + base_text, cursor="arrow")
        elif selected:
            widget.configure(bg=t["ok"], fg=contrast_text(t["ok"]), text=prefix + base_text, cursor="hand2")
        else:
            widget.configure(bg=t["entry_bg"], fg=t["fg"], text=prefix + base_text, cursor="hand2")

    def _refresh_toggle_colors(self, *_args):
        for widget, var, text in self.reg["checks"]:
            self._sync_chip(widget, bool(var.get()), text, "check")
        for widget, var, value, text in self.reg["radios"]:
            self._sync_chip(widget, var.get() == value, text, "radio")

    def _apply_theme(self, name):
        if name not in THEMES:
            return
        self.theme_name = name
        self.theme = t = THEMES[name]
        self.theme_var.set(name)
        self.root.configure(bg=t["root_bg"])
        for w, _role in self.reg["frames"]:
            try:
                w.configure(bg=t["panel_bg"], highlightbackground=t["border"], highlightcolor=t["border"])
            except tk.TclError:
                w.configure(bg=t["panel_bg"])
        for w, role in self.reg["labels"]:
            if role == "readout":
                w.configure(bg=t["readout_bg"], fg=t["readout_fg"])
            else:
                w.configure(bg=t["panel_bg"], fg=t["fg"])
        for w in self.reg["labelframes"]:
            w.configure(bg=t["panel_bg"], fg=t["fg"])
        for w in self.reg["entries"]:
            w.configure(bg=t["entry_bg"], fg=t["entry_fg"], insertbackground=t["entry_fg"],
                        highlightbackground=t["border"], highlightcolor=t["fg"])
        for w in self.reg["scales"]:
            w.configure(bg=t["entry_bg"], fg=t["fg"], troughcolor=t["panel_bg"],
                        highlightbackground=t["panel_bg"], activebackground=t["fg"])
        for w in self.reg["buttons"]:
            w.configure(bg=t["entry_bg"], fg=t["fg"])
        for w in self.reg["canvases"]:
            w.configure(bg=t["panel_bg"])
        for w in self.reg["dropdowns"] + self.reg["custom"]:
            w.apply_theme(t)
        self._refresh_toggle_colors()
        self.exit_btn.configure(bg=t["exit_bg"], fg=t["exit_fg"])
        self.page_host.configure(bg=t["panel_bg"])
        self._update_cal_label()
        self.recording_indicator.itemconfig(self.recording_dot, fill=t["bad"] if self._recording else t["neutral"])
        self._refresh_tolerance_indicator()

        for fig in (self.az_fig, self.el_fig, self.xy_fig):
            fig.set_facecolor(t["plot_bg"])
        for ax in (self.az_ax, self.el_ax, self.xy_ax):
            ax.set_facecolor(t["plot_bg"])
            ax.tick_params(colors=t["plot_fg"], labelsize=7)
            for spine in ax.spines.values():
                spine.set_color(t["plot_fg"])
            ax.grid(True, color=t["grid"], linewidth=0.6)
        self.az_line.set_color(t["plot_fg"])
        self.el_line.set_color(t["plot_fg"])
        self.xy_point.set_color(t["ok"])
        self.xy_source_point.set_color(t["bad"])
        self.xy_connector.set_color(t["marker"])
        self.xy_source_label.set_color(t["bad"])
        self.xy_return_label.set_color(t["ok"])
        self._apply_feed_colors()
        if self.source is not None or hasattr(self, "_loop_after_id"):
            self._refresh_charts()
            self._refresh_feed()

    # (key, label) pairs editable in the color dialog; other keys are
    # inherited from the theme active when the dialog opened
    _CUSTOMIZABLE_KEYS = [
        ("panel_bg", "Panel Background"), ("fg", "Text"), ("entry_bg", "Controls / Fields"),
        ("plot_bg", "Plot Background"), ("plot_fg", "Plot Lines / Text"), ("ok", "Status: OK / Selected"),
        ("bad", "Status: Warning / Off"), ("marker", "Centroid Marker"), ("exit_bg", "Exit Button"),
    ]

    def _open_color_customizer(self):
        base = dict(self.theme)
        t = self.theme
        dialog = tk.Toplevel(self.root)
        dialog.title("Customize Theme Colors")
        dialog.configure(bg=t["panel_bg"])
        dialog.transient(self.root)
        dialog.resizable(False, False)
        tk.Label(dialog, text="Starts from the current theme — change any swatch,\n"
                              "then Apply to switch to a “Custom” theme.",
                 bg=t["panel_bg"], fg=t["fg"], font=("Arial", 10), justify="left").grid(
            row=0, column=0, columnspan=3, sticky="w", padx=10, pady=(10, 6))

        def _btn(parent, text, command):
            lbl = tk.Label(parent, text=text, font=("Arial", 10), relief=tk.RAISED, borderwidth=2,
                           padx=8, pady=3, width=9, cursor="hand2", bg=t["entry_bg"], fg=t["fg"])
            lbl.bind("<ButtonPress-1>", lambda e: lbl.configure(relief=tk.SUNKEN))

            def _release(e):
                lbl.configure(relief=tk.RAISED)
                if 0 <= e.x <= lbl.winfo_width() and 0 <= e.y <= lbl.winfo_height():
                    command()
            lbl.bind("<ButtonRelease-1>", _release)
            return lbl

        swatches = {}
        for i, (key, label) in enumerate(self._CUSTOMIZABLE_KEYS, start=1):
            tk.Label(dialog, text=label, bg=t["panel_bg"], fg=t["fg"], font=("Arial", 10)).grid(
                row=i, column=0, sticky="w", padx=(10, 6), pady=3)
            sw = tk.Label(dialog, text=base[key], bg=base[key], fg=contrast_text(base[key]), width=10,
                          relief=tk.RIDGE, borderwidth=1)
            sw.grid(row=i, column=1, padx=4, pady=3)
            swatches[key] = sw
            _btn(dialog, "Choose…", lambda s=sw: self._pick_custom_color(s)).grid(
                row=i, column=2, padx=(4, 10), pady=3)

        reset_name = self.theme_name if self.theme_name != "Custom" else DEFAULT_THEME
        row = tk.Frame(dialog, bg=t["panel_bg"])
        row.grid(row=len(self._CUSTOMIZABLE_KEYS) + 1, column=0, columnspan=3, pady=(6, 10))

        def _reset():
            for key, sw in swatches.items():
                v = THEMES[reset_name][key]
                sw.configure(bg=v, fg=contrast_text(v), text=v)

        def _apply():
            new = dict(base)
            for key, sw in swatches.items():
                new[key] = sw.cget("bg")
            THEMES["Custom"] = new
            values = list(self.theme_combo["values"])
            if "Custom" not in values:
                self.theme_combo["values"] = values + ["Custom"]
            dialog.destroy()
            self._apply_theme("Custom")

        _btn(row, "Reset", _reset).pack(side=tk.LEFT, padx=4)
        _btn(row, "Apply", _apply).pack(side=tk.LEFT, padx=4)
        _btn(row, "Cancel", dialog.destroy).pack(side=tk.LEFT, padx=4)

    @staticmethod
    def _pick_custom_color(swatch):
        _, hex_color = colorchooser.askcolor(color=swatch.cget("bg"), title="Choose color")
        if hex_color:
            swatch.configure(bg=hex_color, fg=contrast_text(hex_color), text=hex_color)

    # ================================================================
    # System profiles (saved to disk)
    # ================================================================

    def _load_profiles_file(self):
        return read_json(PROFILE_FILE)

    def _write_profiles_file(self):
        try:
            write_json_atomic(PROFILE_FILE, self.system_profiles)
        except OSError as exc:
            messagebox.showerror("System Profile", f"Could not save profiles:\n{exc}")

    # ---------------- settings snapshot (shared by profiles + session) ----------------

    def _calibration_state(self):
        return {"calibrated": bool(self.calibrated), "zero_x_px": float(self.bias_x),
                "zero_y_px": float(self.bias_y), "key": self._cal_key}

    def _system_settings(self):
        """Everything that belongs to one optical system."""
        return {
            "threshold_pct": self.threshold_field.get(),
            "tolerance_arcsec": self.tolerance_field.get(),
            "focal_mm": self.focal_field.get(),
            "exposure_ms": self.exposure_ctl.get(),
            "gain_db": self.gain_ctl.get(),
            "black_level": self.black_ctl.get(),
            "frame_rate": self.fps_ctl.get(),
            "pixel_format": self.pixel_format_var.get(),
            "roi": self.roi_var.get(),
            "flip_x": bool(self.flip_x_var.get()),
            "flip_y": bool(self.flip_y_var.get()),
            "units": self.units_var.get(),
            "calibration": self._calibration_state(),
            "camera_serial": self.source.serial if self.source is not None else None,
        }

    def _apply_system_settings(self, p, push_to_camera):
        """Restore one system's settings. push_to_camera=False is used at
        startup (the values are sent when the camera connects)."""
        self.threshold_field.set(p.get("threshold_pct", self.threshold_field.get()))
        self.tolerance_field.set(p.get("tolerance_arcsec", self.tolerance_field.get()))
        self.focal_field.set(p.get("focal_mm", self.focal_field.get()))
        if p.get("units") in DATA_UNITS:
            self.units_var.set(p["units"])
        for ctl, key in ((self.exposure_ctl, "exposure_ms"), (self.gain_ctl, "gain_db"),
                         (self.black_ctl, "black_level"), (self.fps_ctl, "frame_rate")):
            if key in p:
                if push_to_camera:
                    ctl._apply(p[key])
                else:
                    ctl.set(p[key])
        geometry_changed = (p.get("pixel_format", self.pixel_format_var.get()) != self.pixel_format_var.get()
                            or p.get("roi", self.roi_var.get()) != self.roi_var.get()
                            or bool(p.get("flip_x", False)) != self.flip_x_var.get()
                            or bool(p.get("flip_y", False)) != self.flip_y_var.get())
        if p.get("pixel_format") in [pf for pf, _ in PIXEL_FORMATS]:
            self.pixel_format_var.set(p["pixel_format"])
        if p.get("roi") in ROI_CHOICES:
            self.roi_var.set(p["roi"])
        self.flip_x_var.set(bool(p.get("flip_x", False)))
        self.flip_y_var.set(bool(p.get("flip_y", False)))

        cal = p.get("calibration") or {}
        if cal.get("calibrated"):
            self.bias_x = float(cal.get("zero_x_px", self.bias_x))
            self.bias_y = float(cal.get("zero_y_px", self.bias_y))
            self.calibrated = True
            self._cal_key = cal.get("key")
        elif push_to_camera and "calibration" in p:
            self._clear_calibration()          # profile was saved uncalibrated

        if push_to_camera and geometry_changed and self.source is not None:
            self._apply_geometry_change(pixel_format=self.pixel_format_var.get(), roi=True, flip=True)
        if push_to_camera and self.calibrated and self.source is not None \
                and self._cal_key != self._current_cal_key():
            self._clear_calibration(note="This profile's calibration was made on a different camera or "
                                         "orientation — not applied. Set Calibrated Zero again.")
        self._update_cal_label()
        self._update_plate_scale_label()
        if push_to_camera:
            self._refresh_charts()
            self._refresh_feed()

    # ---------------- system profiles ----------------

    def _save_system_profile(self):
        name = self.system_name_var.get().strip()
        if not name:
            messagebox.showerror("System Profile", "Enter a system name first.")
            return
        self.system_profiles[name] = dict(self._system_settings(),
                                          saved=datetime.now().isoformat(timespec="seconds"),
                                          software_rev=SOFTWARE_REV)
        self._write_profiles_file()
        self.profile_combo["values"] = sorted(self.system_profiles)
        self.profile_var.set(name)
        cal = "with calibration" if self.calibrated else "no calibration"
        self.capture_status_var.set(f"Profile “{name}” saved ({cal})")
        self._save_session()

    def _load_system_profile(self, _event=None):
        name = self.profile_var.get()
        p = self.system_profiles.get(name)
        if not isinstance(p, dict):
            return
        self.system_name_var.set(name)
        self._apply_system_settings(p, push_to_camera=True)
        serial = p.get("camera_serial")
        if serial and self.source is not None and serial != self.source.serial:
            self._note(f"Profile was saved with camera SN {serial}; connected camera is SN {self.source.serial}")
        self.capture_status_var.set(f"Profile “{name}” loaded")
        self._save_session()

    def _remove_system_profile(self):
        name = self.profile_var.get().strip() or self.system_name_var.get().strip()
        if not name or name not in self.system_profiles:
            messagebox.showerror("System Profile", "Select a saved profile to remove first.")
            return
        if not messagebox.askyesno("System Profile", f"Delete the saved profile “{name}”?"):
            return
        del self.system_profiles[name]
        self._write_profiles_file()
        self.profile_combo["values"] = sorted(self.system_profiles)
        self.profile_var.set("")
        self._save_session()

    def _export_profiles(self):
        """Save all profiles to a file (to copy to another computer, or to
        deliver with each unit)."""
        if not self.system_profiles:
            messagebox.showinfo("Export Profiles", "There are no saved profiles yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Export profiles", defaultextension=".json", filetypes=[("Profiles", "*.json")],
            initialdir=self.capture_dir_var.get().strip() or os.path.expanduser("~"),
            initialfile=f"DUV_Profiles_{datetime.now().strftime('%Y%m%d')}.json")
        if not path:
            return
        try:
            write_json_atomic(path, {"duv_profiles": self.system_profiles, "software_rev": SOFTWARE_REV})
            self.capture_status_var.set(f"Exported {len(self.system_profiles)} profile(s)")
        except OSError as exc:
            messagebox.showerror("Export Profiles", f"Could not write the file:\n{exc}")

    def _import_profiles(self):
        path = filedialog.askopenfilename(title="Import profiles", filetypes=[("Profiles", "*.json")])
        if not path:
            return
        data = read_json(path)
        incoming = data.get("duv_profiles", data)
        incoming = {k: v for k, v in incoming.items() if isinstance(v, dict)}
        if not incoming:
            messagebox.showerror("Import Profiles", "No profiles found in that file.")
            return
        clash = sorted(set(incoming) & set(self.system_profiles))
        if clash and not messagebox.askyesno(
                "Import Profiles", "These profiles already exist and will be replaced:\n\n"
                                   + "\n".join(clash) + "\n\nContinue?"):
            return
        self.system_profiles.update(incoming)
        self._write_profiles_file()
        self.profile_combo["values"] = sorted(self.system_profiles)
        self.capture_status_var.set(f"Imported {len(incoming)} profile(s)")

    # ---------------- session: everything, restored at next launch ----------------

    def _session_state(self):
        state = {
            "software_rev": SOFTWARE_REV,
            "theme": self.theme_name,
            "custom_theme": THEMES.get("Custom"),
            "mode": self.mode,
            "source": self.source_var.get(),
            "tab": self.tab_var.get(),
            "sample_rate": self._safe_float(self.rate_var, 10.0),
            "system_name": self.system_name_var.get(),
            "profile": self.profile_var.get(),
            "capture_dir": self.capture_dir_var.get(),
            "instrument_port": self.com_var.get(),
            "test_point": {"enabled": bool(self.test_mode_var.get()), "az": self.test_az_field.get(),
                           "el": self.test_el_field.get(),
                           "origin": [self.test_point_origin_x, self.test_point_origin_y]},
            "window": self.root.geometry(),
        }
        state["system"] = self._system_settings()
        return state

    def _save_session(self):
        """Write the session file (only when something changed)."""
        self._last_autosave = time.perf_counter()
        try:
            state = self._session_state()
            text = json.dumps(state, sort_keys=True)
            if text == self._last_session_json:
                return
            write_json_atomic(SESSION_FILE, state)
            self._last_session_json = text
        except Exception as exc:              # never let saving break the app
            print(f"[session] could not save: {exc}")

    def _restore_session(self, st):
        if not st:
            return
        try:
            if isinstance(st.get("custom_theme"), dict):
                THEMES["Custom"] = dict(THEMES[DEFAULT_THEME], **st["custom_theme"])
            if st.get("theme") in THEMES:
                self.theme_name = st["theme"]
                self.theme = THEMES[self.theme_name]
                self.theme_var.set(self.theme_name)
            if st.get("mode") in MODES:
                self.mode = st["mode"]
                self.mode_var.set(self.mode)
            if st.get("source") in SOURCES:
                self.source_var.set(st["source"])
            if st.get("tab"):
                self.tab_var.set(st["tab"])
            if st.get("sample_rate") in SAMPLE_RATES:
                self.rate_var.set(st["sample_rate"])
            self.system_name_var.set(st.get("system_name", ""))
            if st.get("profile") in self.system_profiles:
                self.profile_var.set(st["profile"])
            if st.get("capture_dir"):
                self.capture_dir_var.set(st["capture_dir"])
            if st.get("instrument_port") in self.serial_ports:
                self.com_var.set(st["instrument_port"])
            tp = st.get("test_point") or {}
            self.test_az_field.set(tp.get("az", 0))
            self.test_el_field.set(tp.get("el", 0))
            if isinstance(tp.get("origin"), list) and len(tp["origin"]) == 2:
                self.test_point_origin_x, self.test_point_origin_y = map(float, tp["origin"])
            self.test_mode_var.set(bool(tp.get("enabled", False)) and st.get("source") == "Simulator")
            if isinstance(st.get("system"), dict):
                self._apply_system_settings(st["system"], push_to_camera=False)
            self._restore_window(st.get("window"))
        except Exception as exc:
            print(f"[session] could not restore everything: {exc}")

    def _restore_window(self, geometry):
        """Reopen at the last size/position, if it still fits this screen."""
        m = re.match(r"(\d+)x(\d+)\+(-?\d+)\+(-?\d+)", geometry or "")
        if not m:
            return
        w, h, x, y = map(int, m.groups())
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        if 600 <= w <= sw and 400 <= h <= sh and 0 <= x <= sw - 100 and 0 <= y <= sh - 100:
            self.root.geometry(f"{w}x{h}+{x}+{y}")


    # ================================================================
    # Capture: screenshot, video, raw frame, CSV log
    # ================================================================

    def _capture_dir(self):
        out_dir = self.capture_dir_var.get().strip() or os.path.expanduser("~")
        os.makedirs(out_dir, exist_ok=True)
        return out_dir

    def _browse_capture_dir(self):
        path = filedialog.askdirectory(initialdir=self.capture_dir_var.get().strip() or os.path.expanduser("~"))
        if path:
            self.capture_dir_var.set(path)

    def _render_feed_image(self):
        """Fallback when a whole-window grab isn't possible: the current
        display image with Source/Return drawn on it."""
        t = self.theme
        s = float(max(1, self.display_step))
        ox, oy = self.display_offset
        if self.last_display_frame is not None:
            img = Image.fromarray(self.last_display_frame).convert("RGB")
        else:
            img = Image.new("RGB", (max(1, int(self.frame_w / s)), max(1, int(self.frame_h / s))),
                            hex_to_rgb(t["plot_bg"]))
        draw = ImageDraw.Draw(img)
        bx, by = (self.bias_x - ox) / s, (self.bias_y - oy) / s
        bad, ok, marker = hex_to_rgb(t["bad"]), hex_to_rgb(t["ok"]), hex_to_rgb(t["marker"])
        draw.line([(0, by), (img.width, by)], fill=bad)
        draw.line([(bx, 0), (bx, img.height)], fill=bad)
        draw.text((bx + 8, by - 14), "Source", fill=bad)
        if self.last_cx is not None:
            cx, cy = (self.last_cx - ox) / s, (self.last_cy - oy) / s
            draw.ellipse([cx - 10, cy - 10, cx + 10, cy + 10], outline=ok, width=2)
            draw.text((cx + 12, cy + 10), "Return", fill=ok)
            draw.line([(bx, by), (cx, cy)], fill=marker)
        return img

    def _capture_window_image(self):
        """Whole-window grab (Windows/macOS). On macOS this needs Screen
        Recording permission for the app running Python (Terminal, VS
        Code, PyCharm...): System Settings > Privacy & Security > Screen
        Recording. Returns (image, error)."""
        try:
            self.root.update_idletasks()
            x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
            w, h = self.root.winfo_width(), self.root.winfo_height()
            return ImageGrab.grab(bbox=(x, y, x + w, y + h)).convert("RGB"), None
        except Exception as exc:
            return self._render_feed_image(), exc

    _PERMISSION_HINT = ("On macOS this is almost always Screen Recording permission: System Settings > "
                        "Privacy & Security > Screen Recording, add the app you run this script from "
                        "(Terminal / VS Code / PyCharm), then restart it.")

    def _take_screenshot(self):
        if not PIL_AVAILABLE:
            messagebox.showerror("Capture", "Screenshots need the Pillow package:\n  python3 -m pip install Pillow")
            return
        try:
            filename = f"DUV_Screenshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            img, err = self._capture_window_image()
            img.save(os.path.join(self._capture_dir(), filename))
            if err is not None:
                self.capture_status_var.set("Saved image only (window grab failed)")
                messagebox.showwarning("Capture", f"Saved the image only — the window grab failed:\n\n{err}\n\n"
                                                  + self._PERMISSION_HINT)
            else:
                self.capture_status_var.set(f"Saved {filename}")
        except OSError as exc:
            messagebox.showerror("Capture", f"Could not save screenshot:\n{exc}")

    def _save_raw_frame(self):
        """Full-resolution, unprocessed camera frame — 8-bit or 16-bit
        TIFF (12-bit data is stored in a 16-bit file, values 0-4095)."""
        if not PIL_AVAILABLE:
            messagebox.showerror("Capture", "Saving frames needs the Pillow package:\n  python3 -m pip install Pillow")
            return
        if self.source is None:
            messagebox.showwarning("Capture", "No camera connected.")
            return
        raw = self.source.latest_raw()
        if raw is None:
            messagebox.showwarning("Capture", "No frame received yet.")
            return
        try:
            arr = np.ascontiguousarray(raw)
            # uint16 -> 16-bit image, uint8 -> 8-bit (mode inferred from dtype)
            img = Image.fromarray(arr if arr.dtype == np.uint8 else arr.astype(np.uint16))
            exp = self.exposure_actual_ms or self.exposure_ctl.get()
            filename = (f"DUV_Frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{arr.shape[1]}x{arr.shape[0]}_"
                        f"{self.pixel_format_var.get()}_{exp:.3f}ms_{self.gain_ctl.get():.1f}dB.tiff")
            img.save(os.path.join(self._capture_dir(), filename))
            self.capture_status_var.set(f"Saved {filename}")
        except Exception as exc:
            messagebox.showerror("Capture", f"Could not save frame:\n{exc}")

    # ---------------- CSV log ----------------

    _CSV_HEADER = ["timestamp", "elapsed_s", "source", "az_arcsec", "el_arcsec", "centroid_x_px", "centroid_y_px",
                   "signal_pct", "peak_counts", "background_counts", "saturated", "spot_px", "exposure_ms",
                   "gain_db", "sensor_temp_c", "calibrated", "zero_x_px", "zero_y_px", "arcsec_per_px",
                   "az_mm_sensor", "el_mm_sensor"]

    def _toggle_csv_log(self):
        if self._csv_file is not None:
            self._close_csv(message=True)
            return
        try:
            filename = f"DUV_Log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            path = os.path.join(self._capture_dir(), filename)
            self._csv_file = open(path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(self._CSV_HEADER)
            self._csv_rows = 0
            self._csv_t0 = time.perf_counter()
            self._csv_name = filename
            self.log_var.set(True)
            self.capture_status_var.set(f"Logging to {filename}")
        except OSError as exc:
            self._csv_file = self._csv_writer = None
            self.log_var.set(False)
            messagebox.showerror("CSV Log", f"Could not create log file:\n{exc}")

    def _write_csv_row(self):
        if self._csv_writer is None:
            return
        f = lambda v, d=4: "" if v is None else f"{v:.{d}f}"   # noqa: E731
        try:
            self._csv_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), f(time.perf_counter() - self._csv_t0, 3),
                self.source_var.get(), f(self.last_az), f(self.last_el), f(self.last_cx, 3), f(self.last_cy, 3),
                self.last_signal, self.last_peak_raw, f(self.last_background, 1), int(self.last_saturated),
                f(self.last_spot_px, 3), f(self.exposure_actual_ms), f(self.gain_ctl.get(), 2), f(self.temp_c, 1),
                int(self.calibrated), f(self.bias_x, 3), f(self.bias_y, 3), f(self._arcsec_per_px(), 6),
                f(self._arcsec_to_mm(self.last_az), 5), f(self._arcsec_to_mm(self.last_el), 5)])
            self._csv_rows += 1
            if self._csv_rows % 10 == 0:
                self._csv_file.flush()
                self.capture_status_var.set(f"Logging to {self._csv_name} ({self._csv_rows} rows)")
        except (OSError, ValueError) as exc:
            self._close_csv()
            messagebox.showerror("CSV Log", f"Logging stopped — could not write:\n{exc}")

    def _close_csv(self, message=False):
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except OSError:
                pass
            if message:
                self.capture_status_var.set(f"Saved {self._csv_name} ({self._csv_rows} rows)")
        self._csv_file = self._csv_writer = None
        self.log_var.set(False)

    # ---------------- screen recording ----------------

    @staticmethod
    def _find_ffmpeg():
        found = shutil.which("ffmpeg")
        if found:
            return found
        for candidate in ("/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"):
            if os.path.exists(candidate):
                return candidate
        return None

    @staticmethod
    def _ffmpeg_screen_device_index(ffmpeg_bin):
        try:
            result = subprocess.run([ffmpeg_bin, "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                                    capture_output=True, text=True, timeout=10)
            for line in result.stderr.splitlines():
                m = re.search(r"\[(\d+)\]\s*Capture screen", line)
                if m:
                    return int(m.group(1))
        except Exception:
            pass
        return None

    def _build_ffmpeg_record_command(self, out_dir, stamp):
        """macOS + ffmpeg: real continuous screen recording cropped to
        this window. Returns (None, None) otherwise (frame fallback)."""
        if platform.system() != "Darwin":
            return None, None
        ffmpeg_bin = self._find_ffmpeg()
        if not ffmpeg_bin:
            return None, None
        device = self._ffmpeg_screen_device_index(ffmpeg_bin)
        if device is None:
            return None, None
        self.root.update_idletasks()
        scale = self.root.winfo_fpixels("1i") / 72.0
        x, y = int(self.root.winfo_rootx() * scale), int(self.root.winfo_rooty() * scale)
        w, h = int(self.root.winfo_width() * scale), int(self.root.winfo_height() * scale)
        w -= w % 2
        h -= h % 2
        if w <= 0 or h <= 0:
            return None, None
        path = os.path.join(out_dir, f"DUV_Recording_{stamp}.mp4")
        cmd = [ffmpeg_bin, "-y", "-f", "avfoundation", "-capture_cursor", "0", "-i", f"{device}:none",
               "-t", str(int(VIDEO_RECORD_SECONDS)), "-vf", f"crop={w}:{h}:{x}:{y}", "-pix_fmt", "yuv420p", path]
        return cmd, path

    def _wait_for_ffmpeg(self):
        try:
            _, stderr = self._ffmpeg_proc.communicate(timeout=VIDEO_RECORD_SECONDS + 20)
        except Exception:
            stderr = b""
        if self.running:
            self.root.after(0, self._on_ffmpeg_done, stderr)

    def _set_recording_ui(self, active):
        self.record_btn.config(text="Stop Recording" if active else "Record Video")
        self.recording_indicator.itemconfig(self.recording_dot,
                                            fill=self.theme["bad"] if active else self.theme["neutral"])

    def _on_ffmpeg_done(self, stderr_bytes):
        self._recording = False
        self._recording_mode = None
        self._ffmpeg_proc = None
        self._set_recording_ui(False)
        path, self._recording_output_path = self._recording_output_path, None
        if path and os.path.exists(path) and os.path.getsize(path) > 1000:
            self.capture_status_var.set(f"Saved {os.path.basename(path)}")
        else:
            err = stderr_bytes.decode(errors="ignore")[-400:] if stderr_bytes else "unknown error"
            self.capture_status_var.set("Recording failed — see popup")
            messagebox.showerror("Capture", f"Screen recording failed:\n\n{err}\n\n{self._PERMISSION_HINT}")

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording()
            return
        if not PIL_AVAILABLE:
            messagebox.showerror("Capture", "Recording needs the Pillow package:\n  python3 -m pip install Pillow")
            return
        try:
            out_dir = self._capture_dir()
        except OSError as exc:
            messagebox.showerror("Capture", f"Could not create output folder:\n{exc}")
            return
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        ffmpeg_cmd, ffmpeg_path = self._build_ffmpeg_record_command(out_dir, stamp)
        self._recording = True
        self._recording_start_time = time.perf_counter()
        self._set_recording_ui(True)
        if ffmpeg_cmd is not None:
            self._recording_mode = "ffmpeg"
            self._recording_output_path = ffmpeg_path
            self.capture_status_var.set("Recording…")
            try:
                self._ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                threading.Thread(target=self._wait_for_ffmpeg, daemon=True).start()
                return
            except Exception:
                self._recording_mode = None
                self._ffmpeg_proc = None
        self._recording_mode = "frames"
        self._recording_frames = []
        self._recording_capture_error = None
        self.capture_status_var.set("Recording…")
        self._recording_after_id = self.root.after(int(VIDEO_RECORD_SECONDS * 1000), self._stop_recording)

    def _stop_recording(self):
        if not self._recording:
            return
        if self._recording_mode == "ffmpeg":
            if self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
                try:
                    self._ffmpeg_proc.terminate()
                except Exception:
                    pass
            return
        self._recording = False
        if self._recording_after_id is not None:
            try:
                self.root.after_cancel(self._recording_after_id)
            except Exception:
                pass
            self._recording_after_id = None
        self._set_recording_ui(False)
        frames, self._recording_frames = self._recording_frames, []
        capture_err, self._recording_capture_error = self._recording_capture_error, None
        if not frames:
            self.capture_status_var.set("Recording stopped — no frames captured.")
            return
        fps = max(1.0, len(frames) / max(0.1, time.perf_counter() - self._recording_start_time))
        try:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            out_dir = self._capture_dir()
            if CV2_AVAILABLE:
                filename = f"DUV_Recording_{stamp}.mp4"
                w, h = frames[0].size
                writer = cv2.VideoWriter(os.path.join(out_dir, filename), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                try:
                    for img in frames:
                        if img.size != (w, h):
                            img = img.resize((w, h))
                        writer.write(np.array(img)[:, :, ::-1])
                finally:
                    writer.release()
            else:
                filename = f"DUV_Recording_{stamp}.gif"
                frames[0].save(os.path.join(out_dir, filename), save_all=True, append_images=frames[1:],
                               duration=max(20, int(1000 / fps)), loop=0)
            self.capture_status_var.set(f"Saved {filename} ({len(frames)} frames)")
            if capture_err is not None:
                messagebox.showwarning("Capture", f"The window grab failed for some frames:\n\n{capture_err}\n\n"
                                                  + self._PERMISSION_HINT)
        except Exception as exc:
            messagebox.showerror("Capture", f"Could not save video:\n{exc}")

    # ================================================================
    # Top-level controls
    # ================================================================

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    def _restart(self):
        """Clear charts, statistics and the calibrated zero."""
        self.paused = False
        self.pause_btn.config(text="Pause")
        self._clear_calibration()
        self._reset_measurement()

    def _exit_app(self):
        if not messagebox.askokcancel("Exit", "Close the DUV Autocollimator Control application?"):
            return
        self._save_session()                  # restored at next launch
        self.running = False
        # cancel pending timers BEFORE destroying the window, otherwise Tk
        # fires them afterwards ('invalid command name ..._update_loop')
        for after_id in (self._loop_after_id, self._startup_after_id, self._recording_after_id,
                         self._auto_exp_after_id):
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception:
                    pass
        self._stop_recording()
        self._close_csv()
        self._disconnect_source(reset=False)
        self.root.quit()
        self.root.destroy()


# ----------------------------------------------------------------------
# Camera bring-up check (no GUI):  --check-camera [--emulator]
# ----------------------------------------------------------------------

def check_camera(use_emulator=False, seconds=6.0):
    """Connect, print everything the GUI relies on, grab frames for a few
    seconds in Mono8 and Mono12, and report frame rate and centroid.
    Run this first when the camera arrives. Exit code 0 = all good."""
    ok = True

    def line(label, value):
        print(f"  {label:<26}{value}")

    try:
        from pypylon import pylon
        print(f"pypylon {getattr(pylon, '__version__', '')} on Python {sys.version.split()[0]} ({sys.executable})")
        devices = list(pylon.TlFactory.GetInstance().EnumerateDevices())
        print("Devices found:")
        for d in devices:
            print(f"  - {d.GetModelName()}  SN {d.GetSerialNumber()}  [{d.GetDeviceClass()}]")
    except ImportError as exc:
        print(f"FAIL  pypylon not importable: {exc}\n      Fix: \"{sys.executable}\" -m pip install pypylon")
        return 1

    src = BaslerSource(use_emulator=use_emulator)
    try:
        src.connect({"pixel_format": "Mono8"})
    except Exception as exc:
        print(f"FAIL  connect: {exc}")
        return 1
    print("\nConnected:")
    line("Model / serial", f"{src.model_name} / {src.serial}" + ("  (EMULATOR)" if src.is_emulated else ""))
    line("Sensor", f"{src.sensor_w} x {src.sensor_h} px  (spec {CAMERA_WIDTH_PX} x {CAMERA_HEIGHT_PX})")
    if not src.is_emulated and (src.sensor_w, src.sensor_h) != (CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX):
        print("  WARN  sensor size differs from the a2A4504 spec")
        ok = False
    line("Pixel formats", ", ".join(src.available_formats) or "?")
    if src.link_speed_bps:
        line("USB link speed", f"{float(src.link_speed_bps) / 1e6:.0f} MB/s")
    for key, unit in (("exposure_ms", "ms"), ("gain_db", "dB"), ("black_level", "DN"), ("frame_rate", "fps")):
        lo, hi = src.limits[key]
        line(f"{key} range", f"{lo:g} .. {hi:g} {unit}")
    t = src.temperature_c()
    line("Sensor temperature", f"{t:.1f} °C" if t is not None else "not available")
    for name in ("ExposureAuto", "GainAuto", "Gamma", "ReverseX", "ReverseY"):
        line(name, src._get(name, "n/a"))
    for w in src.warnings:
        print(f"  WARN  {w}")
        ok = False

    exp = src.set_exposure_ms(EXPOSURE_DEFAULT_MS)
    src.set_frame_rate(CAMERA_MAX_FPS)
    line("Exposure applied", f"{exp:.3f} ms" if exp is not None else "FAILED")
    for fmt in ("Mono8", "Mono12"):
        err = src.reconfigure(pixel_format=fmt, roi_size=None)
        bpp = 1 if src.max_value <= 255 else 2
        bw_fps = USB3_PAYLOAD_BPS / (src.width * src.height * bpp)
        print(f"\nGrabbing {seconds:.0f} s in {src.pixel_format} ({src.width}x{src.height}, "
              f"USB3 limit ~{bw_fps:.1f} fps){'  ERROR ' + err if err else ''}")
        src.start()
        t0, first_id, last = time.perf_counter(), None, None
        while time.perf_counter() - t0 < seconds:
            fid, res = src.get_latest()
            if res is not None:
                first_id = fid if first_id is None else first_id
                last = (fid, res)
            time.sleep(0.05)
        src._stop_grabbing()
        if last is None:
            print("  FAIL  no frames received" + (f" ({src.last_error})" if src.last_error else ""))
            ok = False
            continue
        n = last[0] - first_id + 1
        res = last[1]
        line("Frames processed", f"{n} (~{n / seconds:.1f} fps incl. centroiding)")
        line("Camera ResultingFrameRate", f"{src.resulting_fps():.2f} fps" if src.resulting_fps() else "n/a")
        line("Peak / background", f"{res['peak']:.0f} / {res['background']:.1f} of {src.max_value}"
                                  + ("  SATURATED" if res["saturated"] else ""))
        if res["cx"] is not None:
            line("Centroid (sensor px)", f"{res['cx']:.2f}, {res['cy']:.2f}   spot {res['spot_px']:.2f} px")
        else:
            line("Centroid", "no return detected (expected with the lens capped / no source)")
    src.disconnect()
    print("\nRESULT:", "PASS" if ok else "CHECK WARNINGS ABOVE")
    return 0 if ok else 2


def main():
    if "--check-camera" in sys.argv:
        sys.exit(check_camera(use_emulator="--emulator" in sys.argv))
    root = tk.Tk()
    app = DUVAutocollimatorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app._exit_app)
    if platform.system() == "Darwin":
        # Cmd+Q / app menu Quit go through the same save-and-close path
        root.createcommand("tk::mac::Quit", app._exit_app)
    root.mainloop()


if __name__ == "__main__":
    main()
