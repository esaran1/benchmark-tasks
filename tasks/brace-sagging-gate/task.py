"""Brace a sagging garden gate: fit the loose brace so the gate holds its latch side up by itself.

A small gate frame (pine, 200 x 150 mm) stands upright across the table 33 cm in front of the robots, its face towards
them. The hinge-side stile is fixed to a post; the top and bottom rails are pinned to both stiles, so the frame is a
parallelogram that racks. A steel weight on the latch-side stile has pulled that side down onto a rubber stop: the gate
has sagged. Four V-notch seats stand out from the frame's front face, one in each inside corner, each fixed to the rail
it sits on. A loose pine brace with rounded ends (183.9 mm between its end centres) lies on the table. It fits between
two diagonally opposite seats only when the frame is square. It is a push-only member: it can hold the gate square only
on the diagonal where the gate's weight pushes its ends together (the bottom hinge-side corner to the top latch-side
corner), so the latch side has to be lifted square, the brace fitted on that diagonal, and the gate lowered onto it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
BRACE_STATE=k (0-9; default 0, the task's layout) mirrors the gate, changes the sag and moves the frame and brace;
states 1-9 are the gap test's other starting states (README, Gap test). BRACE_MODE (development only): table (the
task's start, default) | none (no brace) | comp / tens (brace written into the seats of one diagonal, frame square).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Loop, Pin, PartState, env, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("The garden gate has sagged: its latch side has dropped onto the rubber stop. Fit the loose wooden brace "
               "between two of the corner seats so that the gate holds itself up, with the top of the latch-side stile no "
               "more than 3 mm from level with the top of the hinge-side stile, then let go of the gate and the brace.")

# starting states: (hinge side +1 left / -1 right, sag mm, frame x offset mm, brace start (x, y) mm)
_STATES = [(1, 14, 0, (200, 0)), (1, 12, 0, (200, 0)), (1, 16, 0, (200, 0)), (-1, 14, 0, (200, 0)), (-1, 12, 10, (200, 0)),
           (-1, 16, -10, (200, 10)), (1, 14, 10, (190, -10)), (1, 13, -10, (210, 10)), (-1, 15, 0, (190, -10)),
           (1, 15, 5, (200, 20))]
K = int(os.environ.get("BRACE_STATE", "0"))
SGN, _SAG_MM, _DX, _BSTART = _STATES[K]
FX = 0.33 + _DX / 1000                           # frame plane (world x, m)
Y_HINGE, Y_LATCH = SGN * 0.105, -SGN * 0.095      # stile centre lines (pin y), m
Z_BOT, Z_TOP = 0.045, 0.165                       # pin heights above the table, m
W, HGT = abs(Y_HINGE - Y_LATCH), Z_TOP - Z_BOT          # pin spacing along the rails, and between the rails (m)
STILE = (0.015, 0.02, 0.15)                       # x (depth), y (width), z (length)
RAIL = (0.015, 0.02)                              # x (depth), z (height); length spans the stiles
DENS = 500.0                                      # pine, kg/m3
WEIGHT_M = float(os.environ.get("BRACE_WEIGHT", "0.08"))   # latch weight, kg
SAG = float(os.environ.get("BRACE_SAG", str(_SAG_MM / 1000)))          # the latch side rests this far below square on the stop (m)
PIN_DAMP = float(os.environ.get("BRACE_PIN_DAMP", "0.002"))
SEAT_DX = 0.035                                   # seats stand this far out of the frame's front face (m)
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
IN_Y = (Y_HINGE - SGN * STILE[1] / 2, Y_LATCH + SGN * STILE[1] / 2)
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
    m = a + u * SEAT_L * 0.5                          # back plate behind the notch, against the frame face
    out.append(((FACE_X - 0.001, m[0], m[1]), (0.002, SEAT_L * 1.2, SEAT_L * 1.2), PINE, _qx(math.atan2(u[1], u[0]))))
    return out


def _rail_boxes(z: float, keys) -> list:
    y0, y1 = sorted((Y_LATCH - SGN * STILE[1] / 2, Y_HINGE + SGN * STILE[1] / 2))
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
               ((FX + 0.02, Y_HINGE + SGN * 0.02, 0.1), (0.02, 0.02, 0.2), TAN)]          # hinge post behind it
rail_len = W + STILE[1]
HANDLE = (0.012, 0.06, 0.012)                    # x, y (sticking out from the latch stile's outer face), z (m)
HANDLE_Z = 0.10
HANDLE_Y = Y_LATCH - SGN * (STILE[1] / 2 + HANDLE[1] / 2)
rail_m = DENS * RAIL[0] * RAIL[1] * rail_len
stile_m = DENS * _vol(STILE)
PINS = [
    Pin("top_rail", rest=(FX, (Y_HINGE + Y_LATCH) / 2, Z_TOP), anchor=(FX, Y_HINGE, Z_TOP), parent="body",
        boxes=tuple(_rail_boxes(Z_TOP, ("th", "tl"))), axis="X", mass=rail_m, damping=PIN_DAMP, lower=-30, upper=30),
    Pin("latch_stile", rest=(FX, Y_LATCH, (Z_BOT + Z_TOP) / 2), anchor=(FX, Y_LATCH, Z_TOP), parent="top_rail",
        boxes=((( FX, Y_LATCH, (Z_BOT + Z_TOP) / 2), STILE, GREEN),
               ((FX + 0.0175, Y_LATCH, (Z_BOT + Z_TOP) / 2), (0.02, 0.04, 0.02), STEEL),
               ((FX, HANDLE_Y, HANDLE_Z), HANDLE, DARK)),
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


BSTART = (_BSTART[0] / 1000, _BSTART[1] / 1000)


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
    return part("brace", pos=(BSTART[0], BSTART[1], TOP + R_END), mass=BRACE_M, friction=0.6,
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



# ---- measurement ---------------------------------------------------------------------------------------------------
RAIL_REST = {"top_rail": (FX, (Y_HINGE + Y_LATCH) / 2, Z_TOP), "bottom_rail": (FX, (Y_HINGE + Y_LATCH) / 2, Z_BOT)}


def seat_point_local(key: str) -> np.ndarray:
    """Where a brace end's centre sits in corner ``key``'s notch (touching both faces), in its rail's frame (m)."""
    a, b = np.array(CORNERS[key]), np.array(CORNERS[OPPOSITE[key]])
    u = (b - a) / np.linalg.norm(b - a)
    c = a + u * R_END * math.sqrt(2)
    return np.array([BRACE_X, c[0], c[1]]) - np.array(RAIL_REST[SEAT_RAIL[key]])


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def seat_points_world(bodies) -> dict:
    out = {}
    for k in CORNERS:
        p, q = bodies[SEAT_RAIL[k]]
        out[k] = np.asarray(p) + _rot(q) @ seat_point_local(k)
    return out


def brace_ends(st) -> np.ndarray:
    r = st.rotation()
    return np.array([np.asarray(st.pos) + r @ np.array([0.0, s * L_BRACE / 2, 0.0]) for s in (-1, 1)])


def measure(objects, bodies) -> dict:
    """Sag (mm below square), each brace end's nearest seat and its distance (mm), whether the brace sits in two
    diagonally opposite seats (both ends within SEATED_MM), and which diagonal."""
    seats = seat_points_world(bodies)
    rep = {"sag_mm": round(sag_mm(bodies), 2)}
    if "brace" in objects:
        ends = brace_ends(objects["brace"])
        near = []
        for e in ends:
            k = min(seats, key=lambda kk: np.linalg.norm((seats[kk] - e)[1:]))
            d = e - seats[k]
            near.append((k, round(float(np.linalg.norm(d[1:])) * 1000, 2), round(float(d[0]) * 1000, 2)))
        rep["ends"] = near                            # (seat, in-plane distance mm, x offset mm: + towards the frame)
        pair = {near[0][0], near[1][0]}
        x_ok = all(-(SEAT_DX - R_END) * 1000 - 0.5 <= n[2] - X_BACK_MM <= 0.5 for n in near)
        rep["seated"] = (OPPOSITE[near[0][0]] == near[1][0]) and max(n[1] for n in near) <= SEATED_MM and x_ok
        rep["diagonal"] = "comp" if pair == {"bh", "tl"} else ("tens" if pair == {"th", "bl"} else "other")
    return rep


SEATED_MM = 3.0
X_BACK_MM = (FACE_X - 0.002 - R_END - BRACE_X) * 1000   # an end against a back plate sits this far behind BRACE_X


SAMPLE_MM = 2.0                  # points on the robots' collision meshes at most this far apart along a triangle edge
SNAP_MM = 0.1                    # sample points within this lattice step are merged (it moves a point at most 0.09 mm)
# Any point of a mesh face lies within SAMPLE_MM / sqrt(3) (1.15 mm) of a sample point, plus at most 0.09 mm from the
# merging, so a robot surface touching a part has a sample point within 1.25 mm of it: well under RELEASE_MM.
CHUNK_MM = 30.0                  # the points are grouped in cubes this size, each with a bounding sphere, for speed
_ROBOT_PTS: list = []


def _sample_mesh(v: np.ndarray, counts, indices) -> np.ndarray:
    """Points on a mesh's faces (fan-triangulated), on a barycentric grid at most SAMPLE_MM apart along each edge."""
    counts, indices = np.asarray(counts), np.asarray(indices)
    starts = np.r_[0, np.cumsum(counts)[:-1]]
    t = [[], [], []]
    for c in np.unique(counts):
        s = starts[counts == c]
        for j in range(1, c - 1):
            t[0].append(indices[s])
            t[1].append(indices[s + j])
            t[2].append(indices[s + j + 1])
    if not t[0]:
        return np.zeros((0, 3))
    p0, p1, p2 = (v[np.concatenate(k)] for k in t)
    edge = np.max([np.linalg.norm(p1 - p0, axis=1), np.linalg.norm(p2 - p0, axis=1),
                   np.linalg.norm(p2 - p1, axis=1)], axis=0)
    n = np.maximum(1, np.ceil(edge * 1000 / SAMPLE_MM)).astype(int)
    out = []
    for k in np.unique(n):
        sel = n == k
        i_, j_ = np.triu_indices(k + 1)                          # i + j <= k
        a, b = (i_ / k)[None, :, None], ((j_ - i_) / k)[None, :, None]
        q0 = p0[sel][:, None, :]
        out.append((q0 + a * (p1[sel][:, None, :] - q0) + b * (p2[sel][:, None, :] - q0)).reshape(-1, 3))
    return np.concatenate(out)


def _robot_points_local() -> list:
    """For every rigid link of both robots (base, arm links, wrist, the gripper's palm and both fingers): points on its
    collision meshes, about SAMPLE_MM apart, in the link's frame (m), grouped in CHUNK_MM cubes, each with a bounding
    sphere. Read once from the stage (the meshes' placement in their link is fixed)."""
    if _ROBOT_PTS:
        return _ROBOT_PTS
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Usd, UsdGeom, UsdPhysics

    base = env().unwrapped
    stage = get_current_stage()
    cache = UsdGeom.XformCache()
    for key in ("left_robot", "right_robot"):
        names = list(base.scene[key].body_names)
        root = stage.GetPrimAtPath(base.scene[key].cfg.prim_path.replace("env_[^/]+", "env_0"))
        for link in Usd.PrimRange(root):
            if not link.HasAPI(UsdPhysics.RigidBodyAPI) or link.GetName() not in names:
                continue
            pts, it = [], iter(Usd.PrimRange(link))
            for prim in it:
                if prim != link and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    it.PruneChildren()                           # the next link's meshes are its own
                    continue
                if not (prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI)):
                    continue
                mesh = UsdGeom.Mesh(prim)
                m = np.array(cache.ComputeRelativeTransform(prim, link)[0], dtype=np.float64)   # row vectors
                v = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
                v = (np.c_[v, np.ones(len(v))] @ m)[:, :3]
                pts.append(_sample_mesh(v, mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()))
            if not pts:
                continue
            grid = SNAP_MM / 1000
            pts = np.unique(np.round(np.concatenate(pts) / grid) * grid, axis=0)   # merge overlapping hulls' points
            cube = np.floor(pts * 1000 / CHUNK_MM).astype(int)
            _, label = np.unique(cube, axis=0, return_inverse=True)
            label = label.ravel()
            order = np.argsort(label, kind="stable")
            pts, label = pts[order], label[order]
            bounds = np.r_[0, np.flatnonzero(np.diff(label)) + 1, len(pts)]
            centres, radii = [], []
            for a, b in zip(bounds[:-1], bounds[1:]):
                c = (pts[a:b].min(0) + pts[a:b].max(0)) / 2
                centres.append(c)
                radii.append(float(np.linalg.norm(pts[a:b] - c, axis=1).max()))
            _ROBOT_PTS.append((key, link.GetName(), pts, np.array(centres), np.array(radii), bounds))
    print("touch check: " + ", ".join(f"{k}/{n} {len(p)}" for k, n, p, *_ in _ROBOT_PTS) + " points", flush=True)
    return _ROBOT_PTS


def _quat_xyzw_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def robot_shapes() -> list:
    """Every link of both robots where it is now: (link name, its points' local coordinates, rotation, position (m,
    env frame), its chunks' world centres and radii, chunk bounds); [] when offline."""
    environment = env()
    if environment is None:
        return []
    base = environment.unwrapped
    local = _robot_points_local()
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
    poses = {}
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        pos = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy() - origin
        quat = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy()
        poses[key] = (list(art.body_names), pos.astype(np.float64), quat.astype(np.float64))
    out = []
    for key, name, pts, centres, radii, bounds in local:
        names, pos, quat = poses[key]
        r, p = _quat_xyzw_matrix(quat[names.index(name)]), pos[names.index(name)]
        out.append((f"{key}/{name}", pts, r, p, centres @ r.T + p, radii, bounds))
    return out






def _box_gap(world_pts, centre, rot, half) -> float:
    loc = (world_pts - centre) @ rot
    return float(np.linalg.norm(np.maximum(np.abs(loc) - half, 0), axis=1).min())


def _qwxyz(q) -> np.ndarray:
    w, x, y, z = q
    return _rot((x, y, z, w))


def scored_boxes(objects, bodies) -> list:
    """Every box the release check covers, where it is now: (centre, rotation, half-size), m. All of the gate (the
    fixed hinge-side stile and post, both rails with their seats and back plates, the latch-side stile with its weight
    and handle), the stop, and the brace (its bounding box)."""
    out = []
    for c, sz, *_rest in HINGE_BOXES:                      # fixed with the gate's base, which stands at (0, 0, TOP)
        out.append((np.array([c[0], c[1], c[2] + TOP]), np.eye(3), np.array(sz) / 2))
    for pin in PINS:
        p, q = bodies[pin.name]
        r = _rot(q)
        for spec in pin.boxes:
            c, sz = spec[0], spec[1]
            rb = _qwxyz(spec[3]) if len(spec) > 3 else np.eye(3)
            out.append((np.asarray(p) + r @ (np.array(c) - np.array(pin.rest)), r @ rb, np.array(sz) / 2))
    for c, sz, *_rest in STOP.boxes:
        out.append((np.array([FX, Y_LATCH, TOP]) + np.array(c), np.eye(3), np.array(sz) / 2))
    if "brace" in objects:
        b = objects["brace"]
        out.append((np.asarray(b.pos), b.rotation(), np.array([R_END, L_BRACE / 2 + R_END, R_END])))
    return out


def robot_gap_mm(shapes, objects, bodies) -> float:
    """Smallest distance (mm) from any robot point to any box in scored_boxes(); inf when offline."""
    boxes = scored_boxes(objects, bodies)
    best = math.inf
    for _n, pts_l, r, p, centres, radii, bounds in shapes:
        world = pts_l @ r.T + p
        for c, rr, h in boxes:
            if np.min(np.linalg.norm(centres - c, axis=1) - radii) > np.linalg.norm(h) + 0.05:
                continue
            best = min(best, _box_gap(world, c, rr, h) * 1000)
    return best


# ---- score ---------------------------------------------------------------------------------------------------------
RELEASE_MM = 20.0       # counts only with no robot point within this of any part of the gate (frame, seats, weight,
                        # handle, post), the stop or the brace (mm) ...
STILL_MM = 0.2          # ... and the sag changed by no more than this over the last HOLD_STEPS steps (mm)
HOLD_STEPS = 20         # 1 s at 20 Hz; success: full score held this long
SAG_BANDS = ((3.0, 1.0), (6.0, 0.5), (9.0, 0.2))   # (latch side within this of square (mm), score)

_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(step=0, scored=None, last=(0.0, False), sags=[], full_steps=0, report={})


on_reset(_reset_state)
on_step(lambda environment=None: _STATE.__setitem__("step", _STATE["step"] + 1))
_reset_state()


def sag_score(sag: float) -> float:
    for limit, sc in SAG_BANDS:
        if abs(sag) <= limit:
            return sc
    return 0.0


def evaluate(objects, bodies=None, shapes=None) -> dict:
    """Every check, and the score."""
    environment = env()
    if bodies is None:
        if environment is None:
            return {"score": 0.0}
        from partkit import body_poses
        bodies = body_poses(environment, "gate")
    shapes = robot_shapes() if shapes is None else shapes
    rep = measure(objects, bodies)
    sags = _STATE["sags"]
    if _STATE.get("sag_step") != _STATE["step"]:
        _STATE["sag_step"] = _STATE["step"]
        sags.append(rep["sag_mm"])
        del sags[:-(HOLD_STEPS + 1)]
    gap = robot_gap_mm(shapes, objects, bodies)
    rep["robot_mm"] = round(gap, 1) if math.isfinite(gap) else None
    rep["released"] = gap >= RELEASE_MM
    rep["still"] = len(sags) == HOLD_STEPS + 1 and max(sags) - min(sags) <= STILL_MM
    rep["sag_score"] = sag_score(rep["sag_mm"])
    ok = bool(rep.get("seated")) and rep["released"] and rep["still"]
    rep["score"] = rep["sag_score"] if ok else 0.0
    return rep


def _update(objects) -> tuple[float, bool]:
    if _STATE["scored"] == _STATE["step"]:
        return _STATE["last"]
    _STATE["scored"] = _STATE["step"]
    rep = evaluate(objects)
    sc = rep["score"]
    _STATE["full_steps"] = _STATE["full_steps"] + 1 if sc >= 1.0 else 0
    _STATE["report"] = rep
    _STATE["last"] = (sc, _STATE["full_steps"] >= HOLD_STEPS)
    return _STATE["last"]


def success(objects) -> bool:
    """Full score held for 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 unless the brace sits in two diagonally opposite seats (each end within 3 mm of its notch, in the seat's depth),
    no robot point is within 20 mm of any part of the gate (frame, seats, weight, handle, post), the stop or the brace, and
    the latch side moved no more than 0.2 mm over the last second. Then, by how far the latch side sits from square (the
    top of the latch-side stile against the top of the hinge-side stile): within 3 mm 1.0, within 6 mm 0.5, within
    9 mm 0.2, further 0."""
    return _update(objects)[0]
