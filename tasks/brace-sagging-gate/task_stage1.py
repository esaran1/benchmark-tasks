"""Gate-brace prototype scene (stage 1, physics only; not a submission).

A small gate frame stands upright across the table 33 cm in front of the robots, its plane facing them (the y-z plane):
the hinge stile (left, +y) is fixed; the top and bottom rails are pinned to it and to the latch stile (right, -y), so
the frame is a parallelogram that racks. A steel weight on the latch stile makes the latch side drop onto a rubber
stop (the gate has sagged). Four V-notch seats stand out from the frame's front face, one in each inside corner, each
fixed to the rail it sits on. A loose brace with rounded ends fits between two diagonally opposite seats.

BRACE_MODE (development): none | comp (brace written into the bottom-hinge / top-latch seats, frame square) |
tens (brace in the top-hinge / bottom-latch seats, frame square) | table (brace on the table, the task's start).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Loop, Pin, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "prototype"

FX = float(os.environ.get("BRACE_FX", "0.33"))    # frame plane (world x, m)
Y_HINGE, Y_LATCH = 0.105, -0.095                  # stile centre lines (pin y), m
Z_BOT, Z_TOP = 0.045, 0.165                       # pin heights above the table, m
W, HGT = Y_HINGE - Y_LATCH, Z_TOP - Z_BOT          # pin spacing along the rails, and between the rails (m)
STILE = (0.015, 0.02, 0.15)                       # x (depth), y (width), z (length)
RAIL = (0.015, 0.02)                              # x (depth), z (height); length spans the stiles
DENS = 500.0                                      # pine, kg/m3
WEIGHT_M = float(os.environ.get("BRACE_WEIGHT", "0.08"))   # latch weight, kg
SAG = float(os.environ.get("BRACE_SAG", "0.014"))          # the latch side rests this far below square on the stop (m)
PIN_DAMP = float(os.environ.get("BRACE_PIN_DAMP", "0.002"))
SEAT_DX = 0.020                                   # seats stand this far out of the frame's front face (m)
SEAT_L, SEAT_T = 0.022, 0.003                     # V plate length and thickness (m)
R_END = 0.0075                                    # brace end radius (m)
R_SHAFT = 0.006
CLEAR = float(os.environ.get("BRACE_CLEAR", "0.0008"))     # brace end clearance in the square frame (m)
BRACE_M = 0.03
GREEN, TAN, DARK, STEEL, PINE, RUBBER = (0.2, 0.5, 0.25), (0.8, 0.68, 0.45), (0.2, 0.2, 0.22), (0.4, 0.42, 0.45), \
    (0.9, 0.78, 0.55), (0.08, 0.08, 0.08)
FACE_X = FX - STILE[0] / 2                       # the frame's front face
BRACE_X = FACE_X - SEAT_DX / 2                   # the brace's axis plane (centred in the seats' depth)

# inside corners of the square frame (y, z) and the seats' V apexes there
IN_Y = (Y_HINGE - STILE[1] / 2, Y_LATCH + STILE[1] / 2)
IN_Z = (Z_BOT + RAIL[1] / 2, Z_TOP - RAIL[1] / 2)
CORNERS = {"bh": (IN_Y[0], IN_Z[0]), "th": (IN_Y[0], IN_Z[1]), "bl": (IN_Y[1], IN_Z[0]), "tl": (IN_Y[1], IN_Z[1])}
OPPOSITE = {"bh": "tl", "tl": "bh", "th": "bl", "bl": "th"}
SEAT_RAIL = {"bh": "bottom_rail", "bl": "bottom_rail", "th": "top_rail", "tl": "top_rail"}


def _qx(phi: float) -> tuple:
    return (float(math.cos(phi / 2)), float(math.sin(phi / 2)), 0.0, 0.0)


def _seat_boxes(key: str) -> list:
    """The two plates of a 90-degree V notch at corner ``key``, opening along the square frame's diagonal towards
    the opposite corner (world frame, m)."""
    a = np.array(CORNERS[key])
    u = np.array(CORNERS[OPPOSITE[key]]) - a
    u /= np.linalg.norm(u)
    out = []
    for s in (-1, 1):
        ang = s * math.pi / 4
        d = np.array([u[0] * math.cos(ang) - u[1] * math.sin(ang), u[0] * math.sin(ang) + u[1] * math.cos(ang)])
        n = np.array([-d[1], d[0]])
        if n @ u > 0:
            n = -n                                   # away from the inside of the V
        c = a + d * SEAT_L / 2 + n * SEAT_T / 2
        phi = math.atan2(d[1], d[0])                 # plate's long side along d (local y rotated about x)
        out.append(((FACE_X - SEAT_DX / 2, c[0], c[1]), (SEAT_DX, SEAT_L, SEAT_T), PINE, _qx(phi)))
    return out


def _rail_boxes(z: float, keys) -> list:
    y0, y1 = Y_LATCH - STILE[1] / 2, Y_HINGE + STILE[1] / 2
    out = [((FX, (y0 + y1) / 2, z), (RAIL[0], y1 - y0, RAIL[1]), GREEN)]
    for k in keys:
        out += _seat_boxes(k)
    return out


def _vol(size) -> float:
    return float(np.prod(size))


# world positions: the gate part's origin is the world origin of the table top (TOP), so part frame = world - TOP z
def _w(v):
    return (v[0], v[1], v[2])


HINGE_BOXES = [((FX, Y_HINGE, (Z_BOT + Z_TOP) / 2), STILE, GREEN),
               ((FX + 0.02, Y_HINGE + 0.02, 0.1), (0.02, 0.02, 0.2), TAN)]          # hinge post behind it
rail_len = (Y_HINGE + STILE[1] / 2) - (Y_LATCH - STILE[1] / 2)
rail_m = DENS * RAIL[0] * RAIL[1] * rail_len
stile_m = DENS * _vol(STILE)
PINS = [
    Pin("top_rail", rest=(FX, (Y_HINGE + Y_LATCH) / 2, Z_TOP), anchor=(FX, Y_HINGE, Z_TOP), parent="body",
        boxes=tuple(_rail_boxes(Z_TOP, ("th", "tl"))), axis="X", mass=rail_m, damping=PIN_DAMP, lower=-30, upper=30),
    Pin("latch_stile", rest=(FX, Y_LATCH, (Z_BOT + Z_TOP) / 2), anchor=(FX, Y_LATCH, Z_TOP), parent="top_rail",
        boxes=((( FX, Y_LATCH, (Z_BOT + Z_TOP) / 2), STILE, GREEN),
               ((FX + 0.0175, Y_LATCH, (Z_BOT + Z_TOP) / 2), (0.02, 0.04, 0.02), STEEL)),
        axis="X", mass=stile_m + WEIGHT_M, damping=PIN_DAMP, lower=-30, upper=30),
    Pin("bottom_rail", rest=(FX, (Y_HINGE + Y_LATCH) / 2, Z_BOT), anchor=(FX, Y_HINGE, Z_BOT), parent="body",
        boxes=tuple(_rail_boxes(Z_BOT, ("bh", "bl"))), axis="X", mass=rail_m, damping=PIN_DAMP, lower=-30, upper=30),
]
LOOPS = [Loop("latch_bottom", "bottom_rail", "latch_stile", anchor=(FX, Y_LATCH, Z_BOT), axis="X")]
GATE = part("gate", pos=(0.0, 0.0, TOP), mass=0.2, fixed=True, friction=0.6, boxes=HINGE_BOXES, pins=PINS, loops=LOOPS)

STOP_TOP = (Z_BOT + Z_TOP) / 2 - STILE[2] / 2 - SAG      # the latch stile's bottom rests here when sagged
STOP = part("sag_stop", pos=(FX, Y_LATCH, TOP), mass=0.1, fixed=True, friction=0.9,
            boxes=[((0, 0, STOP_TOP / 2), (0.03, 0.03, STOP_TOP), RUBBER)])


def brace_length() -> float:
    """Distance between the brace's end centres: seated in two opposite V notches of the square frame, less CLEAR."""
    a, b = np.array(CORNERS["bh"]), np.array(CORNERS["tl"])
    return float(np.linalg.norm(b - a) - 2 * R_END * math.sqrt(2) - CLEAR)


L_BRACE = brace_length()
MODE = os.environ.get("BRACE_MODE", "table")


def _brace_part():
    if MODE in ("comp", "tens"):                     # spawned seated (square frame): shaft as a rotated box
        key = "bh" if MODE == "comp" else "th"
        a, b = np.array(CORNERS[key]), np.array(CORNERS[OPPOSITE[key]])
        u = (b - a) / np.linalg.norm(b - a)
        mid = (a + b) / 2
        phi = math.atan2(u[1], u[0])
        e = u * L_BRACE / 2
        return part("brace", pos=(float(BRACE_X), float(mid[0]), float(TOP + mid[1])), mass=BRACE_M, friction=0.6,
                    boxes=[((0, 0, 0), (2 * R_SHAFT, L_BRACE, 2 * R_SHAFT), PINE, _qx(phi))],
                    spheres=[((0.0, float(s * e[0]), float(s * e[1])), R_END, PINE) for s in (-1, 1)])
    return part("brace", pos=(0.20, 0.0, TOP + R_END), mass=BRACE_M, friction=0.6,
                boxes=[((0, 0, 0), (0.008, L_BRACE * 0.6, 0.008), PINE)],
                cylinders=[((0, 0, 0), R_SHAFT, L_BRACE, PINE, "Y")],
                spheres=[((0, s * L_BRACE / 2, 0), R_END, PINE) for s in (-1, 1)])


BRACE = _brace_part()
OBJECTS = [GATE, STOP, BRACE] if MODE != "none" else [GATE, STOP]


def seated_pose(key: str):
    """World pose (pos, quat_xyzw) of the brace seated between corner ``key`` and its opposite, square frame."""
    a, b = np.array(CORNERS[key]), np.array(CORNERS[OPPOSITE[key]])
    u = (b - a) / np.linalg.norm(b - a)
    ca = a + u * R_END * math.sqrt(2)
    cb = b - u * R_END * math.sqrt(2)
    mid = (ca + cb) / 2
    phi = math.atan2(u[1], u[0])                     # brace local y onto u, rotating about x
    return (BRACE_X, mid[0], TOP + mid[1]), (math.sin(phi / 2), 0.0, 0.0, math.cos(phi / 2))


def sag_mm(gate_bodies) -> float:
    """How far (mm) the latch stile's pins sit below their square height."""
    p = gate_bodies["latch_stile"][0]
    return float(((Z_BOT + Z_TOP) / 2 + TOP - p[2]) * 1000)


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
