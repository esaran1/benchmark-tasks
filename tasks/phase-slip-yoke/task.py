"""Phase a driveshaft's slip yoke: slide it back onto the splined shaft with its ears in line with the far yoke's ears.

On the station table, about 27 cm in front of the robots, a driveshaft trainer is clamped upright to the bench: a fixed
base plate, the far (tube) yoke at the bottom with its two ears and a cross, a 14 mm shaft tube with a stop collar, and
a six-spline shaft above the collar. Beside it, the slip yoke (a 40 mm splined sleeve with its own two ears on top)
stands upright on the table. The sleeve fits onto the shaft at any of six turned positions, 60 degrees apart; only two
of them (0 and 180 degrees) put its ears in the same plane as the far yoke's ears, which is what makes the output of a
two-joint driveshaft turn evenly with its input.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
DRIVE_STATE=k (0-9; default 0, the task's layout) moves the fixture, turns the far yoke and places the sleeve; states
1-9 are the gap test's other starting states.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import PartState, env, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Slide the slip yoke back onto the driveshaft's splined shaft, all the way down to the collar, turned so "
               "that the shaft's output will turn evenly with its input, then let go of it.")

# starting states: fixture dx, dy (mm), far-yoke ear direction (deg from +x), sleeve start x, y (mm), sleeve ear
# direction relative to the far yoke's (deg; its nearest spline position is out of phase in every state)
_STATES = [(0, 0, 0, 215, 160, 70), (10, 0, 20, 220, 150, 110), (-10, 5, -25, 210, 165, 55), (0, -10, 40, 225, 170, 125),
           (5, 10, -40, 215, 175, 65), (-5, -5, 10, 205, 155, 100), (10, 10, 30, 230, 160, 130), (0, 5, -15, 220, 180, 75),
           (-10, -10, 55, 210, 150, 115), (5, 0, -50, 225, 165, 60)]
K = int(os.environ.get("DRIVE_STATE", "0"))
_DX, _DY, _PHI1, _SX, _SY, _DELTA = _STATES[K]
FX, FY = 0.27 + _DX / 1000, 0.05 + _DY / 1000      # the shaft's axis (world x, y), m
PHI1 = math.radians(_PHI1)                          # far yoke's ear direction about the vertical (rad, from +x)
N_SPLINE = 6

STEEL, DARK, BLUE, ORANGE = (0.62, 0.64, 0.66), (0.25, 0.26, 0.28), (0.15, 0.35, 0.8), (0.95, 0.55, 0.1)

# fixture heights above the table (mm)
BASE_H = 6.0
EAR_Z = (6.0, 36.0)            # far yoke ears
EAR_R = (22.0, 28.0)           # ears' radial span from the axis
EAR_W = 18.0                   # ears' width (tangential)
CROSS_Z, CROSS_R = 24.0, 6.0
TUBE_Z, TUBE_R = (30.0, 66.0), 7.0
COLLAR_Z, COLLAR_R = (62.0, 70.0), 18.0
SPLINE_Z = (70.0, 112.0)       # shaft core and splines
CORE_R = 8.0
KEY_R = (8.0, 13.0)            # each spline's radial span
KEY_W = 6.0                    # each spline's width
# slip yoke (sleeve), in its own frame: origin at the sleeve's bottom centre, ears along +-y
SLEEVE_L = 40.0
RING_R = (13.5, 19.5)          # outer ring (rides on the splines' tips)
BEVEL = 3.0                    # 45-degree lead-in bevel on the ring's inner bottom edge (mm, each way)
LAND_R = (10.5, 13.8)          # the lands between the spline slots ...
LAND_W = 3.5
LAND_Z0 = 6.0                  # ... starting this far above the sleeve's bottom, above the bevel
S_EAR_Z = (40.0, 70.0)
SLEEVE_MASS = 0.042           # kg: the 3D-printed (solid PETG) slip yoke of the real-world build


def _mm(v):
    return tuple(float(x) / 1000 for x in v)


def _qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _radial_box(angle, r0, r1, width, z0, z1, color):
    """A box spanning radius r0..r1 along direction ``angle`` (rad, about z), ``width`` across, z0..z1 (mm)."""
    rc = (r0 + r1) / 2
    return (_mm((rc * math.cos(angle), rc * math.sin(angle), (z0 + z1) / 2)), _mm((r1 - r0, width, z1 - z0)), color,
            _qz(angle))


def _qy(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _radial_diamond(angle, rc, zc, half_diag, width, color):
    """A square bar along the tangential direction at ``angle``, turned 45 degrees so its cross-section (radius,
    height) is a diamond centred at radius rc, height zc, its corners half_diag away (mm): a 45-degree bevel."""
    side = half_diag * math.sqrt(2)
    return (_mm((rc * math.cos(angle), rc * math.sin(angle), zc)), _mm((side, width, side)), color,
            _qmul(_qz(angle), _qy(math.pi / 4)))


def _fixture():
    boxes = [((0, 0, BASE_H / 2000), _mm((80, 80, BASE_H)), DARK)]
    for s in (0.0, math.pi):
        boxes.append(_radial_box(PHI1 + s, *EAR_R, EAR_W, *EAR_Z, BLUE))
    for k in range(N_SPLINE):
        boxes.append(_radial_box(PHI1 + k * 2 * math.pi / N_SPLINE, *KEY_R, KEY_W, *SPLINE_Z, STEEL))
    cyl = [(_mm((0, 0, (TUBE_Z[0] + TUBE_Z[1]) / 2)), TUBE_R / 1000, (TUBE_Z[1] - TUBE_Z[0]) / 1000, STEEL),
           (_mm((0, 0, (COLLAR_Z[0] + COLLAR_Z[1]) / 2)), COLLAR_R / 1000, (COLLAR_Z[1] - COLLAR_Z[0]) / 1000, STEEL),
           (_mm((0, 0, (SPLINE_Z[0] + SPLINE_Z[1]) / 2)), CORE_R / 1000, (SPLINE_Z[1] - SPLINE_Z[0]) / 1000, STEEL)]
    # the cross: a bar through the far yoke's ears and a shorter one across it
    boxes.append(_radial_box(PHI1, -EAR_R[1], EAR_R[1], 2 * CROSS_R, CROSS_Z - CROSS_R, CROSS_Z + CROSS_R, ORANGE))
    boxes.append(_radial_box(PHI1 + math.pi / 2, -16.0, 16.0, 2 * CROSS_R, CROSS_Z - CROSS_R, CROSS_Z + CROSS_R, ORANGE))
    return boxes, cyl


def _sleeve_boxes():
    boxes = []
    seg = 2 * RING_R[0] * math.tan(math.pi / 12) + 1.6
    for k in range(12):                                   # the outer ring, its inner bottom edge bevelled
        a = k * math.pi / 6
        boxes.append(_radial_box(a, *RING_R, seg, BEVEL, SLEEVE_L, ORANGE))
        boxes.append(_radial_box(a, RING_R[0] + BEVEL, RING_R[1], seg, 0.0, BEVEL, ORANGE))
        boxes.append(_radial_diamond(a, RING_R[0] + BEVEL, BEVEL, BEVEL, seg, ORANGE))
    for k in range(N_SPLINE):                             # lands between the spline slots (a slot on each ear line)
        a = math.pi / 2 + math.pi / N_SPLINE + k * 2 * math.pi / N_SPLINE
        boxes.append(_radial_box(a, *LAND_R, LAND_W, LAND_Z0, SLEEVE_L, ORANGE))
    for s in (1, -1):                                     # the two ears, along +-y
        boxes.append(_radial_box(s * math.pi / 2, *EAR_R, EAR_W, *S_EAR_Z, ORANGE))
    return boxes


_fb, _fc = _fixture()
FIXTURE = part("shaft", pos=(FX, FY, TOP), mass=2.0, fixed=True, friction=0.4, boxes=_fb, cylinders=_fc)
SLEEVE_YAW0 = PHI1 + math.radians(_DELTA) - math.pi / 2          # part yaw so its ears (+y) point at PHI1 + DELTA
SLEEVE = part("slip_yoke", pos=(_SX / 1000, _SY / 1000, TOP + 0.0005), yaw=SLEEVE_YAW0, mass=SLEEVE_MASS,
              friction=0.4, boxes=_sleeve_boxes())
OBJECTS = [FIXTURE, SLEEVE]

# -- measurement -----------------------------------------------------------------------------------------------------
SEAT_MM = 3.0           # seated: the sleeve's bottom within this above the collar's top (mm) ...
AXIS_MM = 3.0           # ... its axis within this of the shaft's axis at the collar (mm) ...
TILT_MAX = 5.0          # ... and within this of vertical (degrees)
ENGAGED_MM = 15.0       # on the shaft: at least this much of the sleeve below the splines' top (mm)
PHASE_TOL = 10.0        # in phase: its ears within this of the far yoke's ear line (degrees)
RELEASE_MM = 5.0        # counts only with no robot point within this of the sleeve (mm) ...
STILL_MM = 1.0          # ... and its bottom centre moved less than this over the last HOLD_STEPS steps (mm)
HOLD_STEPS = 20         # 1 s at 20 Hz; success: full score held this long


def sleeve_eval(st: PartState) -> dict:
    r = st.rotation()
    bottom = np.asarray(st.pos, float)
    axis = r[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2])))))
    ear = r[:, 1]
    ear_dir = math.degrees(math.atan2(ear[1], ear[0]))
    d = (ear_dir - math.degrees(PHI1)) % 180.0
    phase = min(d, 180.0 - d)                                  # 0..90 degrees off the far yoke's ear line
    height = (bottom[2] - TOP) * 1000 - COLLAR_Z[1]            # sleeve bottom above the collar's top (mm)
    off = math.hypot(bottom[0] - FX, bottom[1] - FY) * 1000
    engaged = SPLINE_Z[1] - COLLAR_Z[1] - height               # length of sleeve below the splines' top (mm)
    seated = bool(height <= SEAT_MM and off <= AXIS_MM and tilt <= TILT_MAX)
    on_shaft = bool(engaged >= ENGAGED_MM and off <= AXIS_MM and tilt <= TILT_MAX)
    return {"height_mm": round(height, 2), "axis_off_mm": round(off, 2), "tilt_deg": round(tilt, 2),
            "phase_deg": round(phase, 2), "engaged_mm": round(engaged, 1), "seated": seated, "on_shaft": on_shaft}


def placement_score(e: dict) -> float:
    if e["phase_deg"] > PHASE_TOL:
        return 0.0
    if e["seated"]:
        return 1.0
    if e["on_shaft"]:
        return 0.5
    return 0.0


# -- robot points (release check) -------------------------------------------------------------------------------
SAMPLE_MM = 2.0                  # points on the robots' collision meshes at most this far apart along a triangle edge
SNAP_MM = 0.1                    # sample points within this lattice step are merged (it moves a point at most 0.09 mm)
# Any point of a mesh face lies within SAMPLE_MM / sqrt(3) (1.15 mm) of a sample point, plus at most 0.09 mm from the
# merging, so a robot surface touching the sleeve has a sample point within 1.25 mm of it: well under RELEASE_MM.
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






def robot_to_box_mm(shapes, st: PartState, size_mm) -> float:
    """The smallest distance (mm) from any point of either robot to a box part (0 inside it); inf when offline."""
    best = math.inf
    rc, c = st.rotation(), np.asarray(st.pos)
    half = np.array(size_mm) / 2000
    for _name, pts_l, r, p, centres, radii, bounds in shapes:
        near = np.flatnonzero(np.linalg.norm(centres - c, axis=1) <= radii + np.linalg.norm(half) + 0.02)
        if not len(near):
            continue
        world = np.concatenate([pts_l[bounds[k]:bounds[k + 1]] for k in near]) @ r.T + p
        loc = (world - c) @ rc
        best = min(best, float(np.linalg.norm(np.maximum(np.abs(loc) - half, 0), axis=1).min()) * 1000)
    return best


def _quat_wxyz_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


_SLEEVE_BOXES = [(np.array(c, float), np.array(sz, float) / 2, _quat_wxyz_matrix(q)) for c, sz, _col, q in _sleeve_boxes()]


def robot_to_part_mm(shapes, st: PartState) -> float:
    """The smallest distance (mm) from any point of either robot to any box of the slip yoke; inf when offline."""
    best = math.inf
    rc, c = st.rotation(), np.asarray(st.pos)
    for _name, pts_l, r, p, centres, radii, bounds in shapes:
        near = np.flatnonzero(np.linalg.norm(centres - (c + rc @ [0, 0, 0.035]), axis=1) <= radii + 0.05 + 0.02)
        if not len(near):
            continue
        world = np.concatenate([pts_l[bounds[k]:bounds[k + 1]] for k in near]) @ r.T + p
        loc = (world - c) @ rc                                   # sleeve frame
        for bc, half, br in _SLEEVE_BOXES:
            q = (loc - bc) @ br
            best = min(best, float(np.linalg.norm(np.maximum(np.abs(q) - half, 0), axis=1).min()) * 1000)
    return best


_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(step=0, scored=None, last=(0.0, False), recent=[], full_steps=0, report={})


@on_step
def _count_step(environment=None) -> None:
    _STATE["step"] += 1


_reset_state()


def evaluate(objects, shapes=None) -> dict:
    st = objects[SLEEVE.name]
    e = sleeve_eval(st)
    recent = _STATE["recent"]
    if _STATE.get("recent_step") != _STATE["step"]:
        _STATE["recent_step"] = _STATE["step"]
        recent.append(np.asarray(st.pos, dtype=float).copy())
        del recent[:-(HOLD_STEPS + 1)]
    e["still"] = bool(len(recent) == HOLD_STEPS + 1 and float(np.linalg.norm(recent[-1] - recent[0])) * 1000 < STILL_MM)
    e["placement"] = placement_score(e)
    # the robots' distance only matters once the placement scores: skipped before that (it is the slow part)
    gap = robot_to_part_mm(robot_shapes() if shapes is None else shapes, st) if e["placement"] > 0 else math.nan
    e["robot_mm"] = round(gap, 1) if math.isfinite(gap) else None
    e["released"] = bool(gap >= RELEASE_MM) if e["placement"] > 0 else None
    e["score"] = e["placement"] if (e["released"] and e["still"]) else 0.0
    return e


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
    """0 unless the slip yoke's ears are within 10 degrees of the far yoke's ear line (either way round), no robot point
    is within 5 mm of it, and it moved less than 1 mm over the last second. Then 1.0 if it is seated (its bottom within
    3 mm of the collar's top, its axis within 3 mm of the shaft's axis and within 5 degrees of vertical); 0.5 if it is
    on the shaft but not down to the collar (at least 15 mm of it below the splines' top); otherwise 0."""
    return _update(objects)[0]
