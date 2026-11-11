"""Reverse a backwards-running three-phase conveyor motor by swapping two of its supply leads.

A short belt conveyor stands across the back of the table, driven by a three-phase motor in front of it. The motor's
terminal box faces the robots: three plug-in sockets U, V and W (left to right, at table level) hold the plugs of three
supply leads (brown L1, black L2, grey L3) that come from a cable gland at the front of the table. The motor was just
replaced and the conveyor now runs backwards. Swapping any two leads reverses a three-phase motor; moving all three one
socket along (a cyclic shift) keeps the phase sequence and changes nothing.

The supply is off at the start. Once the robot has touched a plug or lead, the "tester" switches the supply on whenever
all three plugs are home in three sockets, no robot point is within RELEASE_MM of the terminal box, the gland or any
lead, and the plugs have been still, for 1 s. The motor then turns the
way the lead order sets: forward for an odd permutation of the starting order, backwards for an even one. The direction
is computed from the permutation, not simulated electrically. The conveyor's tote and a pointer disc on the drive show
it: the tote runs past the green (forward) or the red (backward) line.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
MOTOR_STATE=k (0-9; default 0, the task's layout) selects a gap-test starting state (README, Gap test).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the leads are simulated cables)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from partkit import Slider, Pin, env, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("This conveyor's motor was just replaced and now runs backwards. Rearrange the supply leads among the "
               "terminal box's three sockets so that the conveyor runs the way the arrows on its guard point, with every "
               "plug pushed fully home, then let go. The supply is isolated; it is switched on once you are clear.")

# -- starting states: (lead in sockets U, V, W as lead indices, lead slack bow side +1 left / -1 right,
#    drive offset x, y (mm), drive yaw (deg)); state 0 is the task's layout ----------------------------------------
_STATES = [((0, 1, 2), 1, 0, 0, 0), ((1, 0, 2), -1, 10, 0, 0), ((2, 1, 0), 1, -10, 5, 3), ((0, 2, 1), -1, 0, -10, -5),
           ((1, 2, 0), 1, 5, 10, 5), ((2, 0, 1), -1, -5, -5, -3), ((0, 1, 2), -1, 10, -10, 4), ((2, 1, 0), 1, 0, 10, -4),
           ((1, 0, 2), 1, -10, 0, 2), ((0, 2, 1), -1, 5, 5, -2)]
K = int(os.environ.get("MOTOR_STATE", "0"))
START, BOW, _DX, _DY, _YAW = _STATES[K]

# -- colours -----------------------------------------------------------------------------------------------------------
BROWN, BLACK, GREY = (0.42, 0.24, 0.10), (0.06, 0.06, 0.06), (0.58, 0.59, 0.60)
MOTOR_BLUE, FRAME_GREY, BELT_BLACK = (0.30, 0.42, 0.55), (0.72, 0.73, 0.74), (0.10, 0.10, 0.10)
YELLOW, GREEN, RED, WHITE, TOTE_BLUE, DARK = ((0.95, 0.78, 0.10), (0.10, 0.65, 0.20), (0.85, 0.12, 0.10),
                                              (0.95, 0.95, 0.95), (0.12, 0.35, 0.85), (0.22, 0.22, 0.24))

# -- leads ------------------------------------------------------------------------------------------------------------
LEADS = ("brown", "black", "grey")          # L1, L2, L3
LEAD_COLOR = {"brown": BROWN, "black": BLACK, "grey": GREY}
PL, PW, PH = 0.055, 0.016, 0.014             # plug: length along the lead, width, height (m)
PLUG_M = 0.015
R = 0.003                                    # 6 mm lead
LEAD_LEN = 0.24                              # gland to plug (m)
SEG = 0.02
BEND_K, BEND_D = 0.01, 0.001                 # lead bending spring (N m/rad) and damping per joint; the spring's
                                             # rest shape follows the lead's current shape (plastic=True)
DEPTH0 = 0.024                               # plug tips start this far inside their sockets (m)

# -- drive: terminal box + motor + conveyor, one fixed part; its frame is the socket face's centre on the table, x
#    into the box, y left -----------------------------------------------------------------------------------------
FACE_X = 0.36 + _DX / 1000                   # socket face (world x at state 0's yaw)
FACE_Y = _DY / 1000
YAW = math.radians(_YAW)
CAV_W, CAV_H, CAV_D = PW + 0.006, PH + 0.005, 0.026    # socket opening (y, z) and depth (x): 22 x 19 x 26 mm
PITCH = 0.05                                 # socket spacing (m); U at +PITCH, V at 0, W at -PITCH (box y)
SOCKETS = ("U", "V", "W")
SOCKET_Y = (PITCH, 0.0, -PITCH)
BOX_D, BOX_W, BOX_H = 0.06, 0.18, 0.045      # terminal box depth (x), width (y), height (z)
MOTOR_X, MOTOR_R, MOTOR_L = 0.10, 0.04, 0.14  # motor body: centre x, radius, length along y
BED_X, BED_W, BED_L, BED_H = 0.27, 0.10, 0.40, 0.05   # conveyor bed: centre x, width (x), length (y), belt top z
GUARD_X0, GUARD_X1, GUARD_Z = 0.17, 0.22, 0.052        # yellow guard plate in front of the belt (x range, top z)
TOTE = (0.06, 0.08, 0.04)                    # tote: x, y, z (m)
TOTE_M = 0.15
LINE_Y = 0.06                                # green (forward, -y) and red (backward, +y) lines across the guard (m)
FWD_MOVE = LINE_Y + TOTE[1] / 2              # the tote is wholly past a line once it has moved this far (m)
TOTE_STOP = 0.13                             # the tote's travel each way (m)
TOTE_SPEED = 0.10                            # belt speed when powered (m/s)
GEAR = (0.07, 0.13, -0.15, -0.09, 0.08)      # right-angle gearbox block: x0, x1, y0, y1, height
DISC_R, DISC_W = 0.025, 0.004                # pointer disc on the gearbox's front face (axis x)
DISC_C = (GEAR[0] - DISC_W / 2 - 0.001, (GEAR[2] + GEAR[3]) / 2, 0.045)
DISC_SPEED = 3.0                             # rad/s when powered


def _yaw_q(a: float) -> tuple:
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _socket_box_boxes() -> list:
    """The terminal box: solid back, a roof over the sockets, and walls between and beside them (box frame)."""
    c = MOTOR_BLUE
    out = [((CAV_D + (BOX_D - CAV_D) / 2, 0.0, BOX_H / 2), (BOX_D - CAV_D, BOX_W, BOX_H), c),
           ((CAV_D / 2, 0.0, (CAV_H + BOX_H) / 2), (CAV_D, BOX_W, BOX_H - CAV_H), c)]
    edges = sorted([-BOX_W / 2, BOX_W / 2] + [y + s * CAV_W / 2 for y in SOCKET_Y for s in (-1, 1)])
    for y0, y1 in zip(edges[0::2], edges[1::2]):
        out.append(((CAV_D / 2, (y0 + y1) / 2, CAV_H / 2), (CAV_D, y1 - y0, CAV_H), c))
    return out


def _arrow(x: float, y0: float, y1: float, z: float, w: float, color) -> list:
    """A flat arrow from y0 to y1 (pointing at y1) lying at height z, shaft width w."""
    s = 1.0 if y1 > y0 else -1.0
    head = 0.03
    out = [((x, (y0 + y1 - s * head) / 2, z), (w, abs(y1 - y0) - head, 0.002), color)]
    for side in (-1, 1):
        a = side * math.radians(35) * s
        out.append(((x + side * 0.009, y1 - s * 0.013, z), (w, 0.032, 0.002), color, _yaw_q(a)))
    return out


BOX_BOXES = _socket_box_boxes()
DRIVE_BOXES = BOX_BOXES + [
    ((BED_X, 0.0, (BED_H - 0.006) / 2), (BED_W, BED_L, BED_H - 0.006), FRAME_GREY),          # bed frame
    ((BED_X, 0.0, BED_H - 0.003), (BED_W - 0.01, BED_L, 0.006), BELT_BLACK),                # belt
    (((GUARD_X0 + GUARD_X1) / 2, 0.0, GUARD_Z / 2), (GUARD_X1 - GUARD_X0, BED_L, GUARD_Z), YELLOW),   # guard plate
    *[(((GUARD_X0 + GUARD_X1) / 2, s * LINE_Y, GUARD_Z + 0.001), (GUARD_X1 - GUARD_X0, 0.006, 0.002), col)
      for s, col in ((-1, GREEN), (1, RED))],
    *_arrow((GUARD_X0 + GUARD_X1) / 2, 0.16, 0.08, GUARD_Z + 0.001, 0.006, BELT_BLACK),   # arrow: forward is -y
    *_arrow((GUARD_X0 + GUARD_X1) / 2, -0.08, -0.16, GUARD_Z + 0.001, 0.006, BELT_BLACK),
    (((GEAR[0] + GEAR[1]) / 2, (GEAR[2] + GEAR[3]) / 2, GEAR[4] / 2), (GEAR[1] - GEAR[0], GEAR[3] - GEAR[2], GEAR[4]),
     DARK),
]
DRIVE_CYLS = [((MOTOR_X, 0.0, MOTOR_R), MOTOR_R, MOTOR_L, MOTOR_BLUE, "Y"),
              ((MOTOR_X, -MOTOR_L / 2 - 0.006, MOTOR_R), 0.01, 0.012, DARK, "Y"),                 # coupling to gearbox
              ((GEAR[1] + (BED_X - BED_W / 2 - GEAR[1]) / 2, (GEAR[2] + GEAR[3]) / 2, 0.035), 0.01,
               BED_X - BED_W / 2 - GEAR[1], DARK, "X")]                                            # shaft into the bed
TOTE_REST = (BED_X, 0.0, BED_H + TOTE[2] / 2 + 0.0005)
DRIVE = part("drive", pos=(FACE_X, FACE_Y, TOP), yaw=YAW, mass=2.0, fixed=True, friction=0.8,
             boxes=DRIVE_BOXES, cylinders=DRIVE_CYLS,
             sliders=[Slider("tote", rest=TOTE_REST, boxes=((TOTE_REST, TOTE, TOTE_BLUE),), travel=TOTE_STOP,
                             overtravel=TOTE_STOP, rate=4000.0, damping=60.0, mass=TOTE_M, axis="Y", sign=-1)],
             pins=[Pin("disc", rest=DISC_C, anchor=DISC_C, axis="X", mass=0.01, damping=0.05, lower=-1e5, upper=1e5,
                       cylinders=((DISC_C, DISC_R, DISC_W, WHITE, "X"),),
                       boxes=(((DISC_C[0] - DISC_W / 2 - 0.0008, DISC_C[1], DISC_C[2] + DISC_R / 2 - 0.002),
                               (0.0015, 0.007, DISC_R - 0.002), RED),))])

# -- gland: the supply cable's gland plate at the front of the table (world frame, fixed) ------------------------------
GLAND_X = 0.15                               # its back face; the leads leave it here
GLAND_SIZE = (0.03, 0.16, 0.03)
GLAND = part("gland", pos=(GLAND_X - GLAND_SIZE[0] / 2, 0.0, TOP), mass=0.5, fixed=True,
             boxes=[((0, 0, GLAND_SIZE[2] / 2), GLAND_SIZE, DARK)])
LEAD_Z = TOP + PH / 2 + 0.0005               # lead and plug centre height at spawn (plugs rest on the table)


def drive_rot() -> np.ndarray:
    c, s = math.cos(YAW), math.sin(YAW)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def to_world(p_local) -> np.ndarray:
    """Drive frame -> world (m)."""
    return np.array([FACE_X, FACE_Y, TOP]) + drive_rot() @ np.asarray(p_local, float)


def to_drive(p_world) -> np.ndarray:
    return drive_rot().T @ (np.asarray(p_world, float) - np.array([FACE_X, FACE_Y, TOP]))


def _bends(start_xy, end_xy, end_heading: float, side: int, n: int) -> tuple:
    """Joint bends for a lead of n SEG-long segments from start_xy (heading 0, +x) to end_xy (heading end_heading): the
    segment headings are end_heading * t + a sin(2 pi t) + b sin(pi t), t = i / (n - 1), solved for a, b so the lead's
    end lands on end_xy; ``side`` picks the slack bow to the left (+1) or right (-1)."""
    t = np.arange(n) / (n - 1)
    target = np.asarray(end_xy, float) - np.asarray(start_xy, float)

    def end(ab):
        h = end_heading * t + ab[0] * np.sin(2 * np.pi * t) + ab[1] * np.sin(np.pi * t)
        return SEG * np.array([np.cos(h).sum(), np.sin(h).sum()]), h

    def err(ab):
        return end(ab)[0] - target

    grid = [np.array([a, b]) for a in np.linspace(0.05, 1.5, 30) * side for b in np.linspace(-1.0, 1.0, 41)]
    ab = min(grid, key=lambda v: np.linalg.norm(err(v)))      # bow on ``side``: a has its sign
    for _ in range(50):                      # damped Newton on (a, b) from the best grid point
        f = err(ab)
        if np.linalg.norm(f) < 1e-10:
            break
        J = np.zeros((2, 2))
        for k in range(2):
            d = np.zeros(2)
            d[k] = 1e-7
            J[:, k] = (err(ab + d) - f) / 1e-7
        ab = ab - np.clip(np.linalg.solve(J, f), -0.1, 0.1)
    if np.linalg.norm(err(ab)) > 1e-7 or np.sign(ab[0]) != side:
        raise ValueError(f"lead shape not solved: {ab}, {err(ab)}")
    _f, h = end(ab)
    return tuple(float(v) for v in np.diff(h))


def socket_mouth(j: int) -> np.ndarray:
    """World position of socket j's opening centre on the table (m)."""
    return to_world((0.0, SOCKET_Y[j], 0.0))


def _lead(i: int):
    j = START.index(i)                       # the socket it starts in; the gland slot lines up with it
    a = np.array([GLAND_X + 0.001, SOCKET_Y[j] * 1.0, LEAD_Z])
    back = to_world((-(PL - DEPTH0), SOCKET_Y[j], 0.0))   # the plug's back end when DEPTH0 deep
    n = round(LEAD_LEN / SEG)
    return cable(f"lead_{LEADS[i]}", start=tuple(a.tolist()), yaw=0.0, length=LEAD_LEN, radius=R, segment=SEG,
                 anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=PLUG_M, color=LEAD_COLOR[LEADS[i]]),
                 color=LEAD_COLOR[LEADS[i]], bend_stiffness=BEND_K, bend_damping=BEND_D, plastic=True,
                 bends=_bends(a[:2], back[:2], YAW, BOW, n))


LEAD_OBJS = [_lead(i) for i in range(3)]
OBJECTS = [DRIVE, GLAND, *LEAD_OBJS]


# ---- measurement ---------------------------------------------------------------------------------------------------
IN_MM = 8.0              # a plug is in a socket when lined up with it and its tip at least this deep (mm)
HOME_MM = 15.0           # ... and home (fully pushed in) at least this deep (mm)
LINE_MM = 8.0            # lined up: plug centre within this of the socket's centre line sideways (mm) ...
LINE_DEG = 25.0          # ... and its axis within this of the socket's axis (deg)


def plug_pose(st):
    """(centre, unit axis pointing out through the plug's tip) of a lead's plug, world m."""
    p = np.asarray(st.end_plug, float)
    a = p - np.asarray(st.points[-1], float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def socket_of(st) -> tuple:
    """(socket index or None, tip depth mm) for one lead's plug: the socket it is lined up with, and how far its tip is
    inside the socket face (negative: outside)."""
    p, a = plug_pose(st)
    if not np.isfinite(p).all():
        return None, -999.0
    pl, al = to_drive(p), drive_rot().T @ a
    tip = pl + al * PL / 2
    if al[0] < math.cos(math.radians(LINE_DEG)) or pl[2] > CAV_H + 0.005:
        return None, round(float(tip[0]) * 1000, 2)
    j = int(np.argmin([abs(pl[1] - y) for y in SOCKET_Y]))
    if abs(pl[1] - SOCKET_Y[j]) * 1000 > LINE_MM:
        return None, round(float(tip[0]) * 1000, 2)
    return j, round(float(tip[0]) * 1000, 2)


def parity(perm) -> int:
    """0 for an even permutation of (0, 1, 2), 1 for an odd one."""
    p = list(perm)
    inv = sum(1 for x in range(3) for y in range(x + 1, 3) if p[x] > p[y])
    return inv % 2


def measure(objects) -> dict:
    """Which socket each lead's plug is in and how deep; whether all three are in different sockets (in: >= IN_MM;
    home: >= HOME_MM); the order's parity relative to the start (odd = reversed = forward)."""
    rep = {"leads": {}}
    where = []
    for i, name in enumerate(LEADS):
        j, d = socket_of(objects[f"lead_{name}"])
        rep["leads"][name] = (SOCKETS[j] if j is not None and d >= IN_MM else None, d)
        where.append(j if (j is not None and d >= IN_MM) else None)
    rep["all_in"] = None not in where and len(set(where)) == 3
    rep["all_home"] = rep["all_in"] and all(rep["leads"][n][1] >= HOME_MM for n in LEADS)
    if rep["all_in"]:
        # lead i went from socket START.index(i) to where[i]; parity of that map on the sockets
        perm = [None] * 3
        for i in range(3):
            perm[START.index(i)] = where[i]
        rep["odd"] = bool(parity(perm))
    else:
        rep["odd"] = None
    return rep


# ---- robot points (release check) ----------------------------------------------------------------------------------
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
    """For every rigid link of both robots: points on its collision meshes, about SAMPLE_MM apart, in the link's frame
    (m), grouped in CHUNK_MM cubes, each with a bounding sphere. Read once from the stage."""
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


def scored_boxes(objects) -> list:
    """Boxes the release check covers, where they are now: (centre, rotation, half-size), m: every box of the terminal
    box, the gland, and each lead's plug."""
    rot = drive_rot()
    out = [(to_world(c), rot, np.array(s) / 2) for c, s, *_r in BOX_BOXES]
    out.append((np.array([GLAND_X - GLAND_SIZE[0] / 2, 0.0, TOP + GLAND_SIZE[2] / 2]), np.eye(3),
                np.array(GLAND_SIZE) / 2))
    for name in LEADS:
        p, a = plug_pose(objects[f"lead_{name}"])
        z = np.array([0.0, 0.0, 1.0])
        y = np.cross(z, a)
        y = y / (np.linalg.norm(y) + 1e-12)
        out.append((p, np.c_[a, y, np.cross(a, y)], np.array([PL, PW, PH]) / 2))
    return out


def scored_capsules(objects) -> list:
    """Each lead's cable as segments (p0, p1, radius), m: from the gland through every segment centre to the plug."""
    out = []
    for name in LEADS:
        st = objects[f"lead_{name}"]
        poly = st.polyline()
        out += [(poly[k], poly[k + 1], R) for k in range(len(poly) - 1)]
    return out


def _box_gap(world_pts, centre, rot, half) -> np.ndarray:
    loc = (world_pts - centre) @ rot
    return np.linalg.norm(np.maximum(np.abs(loc) - half, 0), axis=1)


def _seg_gap(world_pts, p0, p1, r) -> np.ndarray:
    d = p1 - p0
    t = np.clip(((world_pts - p0) @ d) / max(float(d @ d), 1e-12), 0.0, 1.0)
    return np.maximum(np.linalg.norm(world_pts - (p0 + t[:, None] * d), axis=1) - r, 0.0)


def robot_gaps_mm(shapes, objects) -> tuple:
    """Smallest distance (mm) from any robot point to (the terminal box, the gland or any lead) and to the leads alone
    (plug boxes and cables); inf when no robot point is within 30 mm of their bounding box (and offline). Each stops
    early at 0 (touching)."""
    boxes, caps = scored_boxes(objects), scored_capsules(objects)
    n_fixed = len(boxes) - len(LEADS)                 # scored_boxes lists the fixed boxes first, then the plugs
    lo = np.min([c - np.abs(r) @ h for c, r, h in boxes] + [np.minimum(a, b) - rr for a, b, rr in caps], axis=0)
    hi = np.max([c + np.abs(r) @ h for c, r, h in boxes] + [np.maximum(a, b) + rr for a, b, rr in caps], axis=0)
    best_all = best_lead = math.inf
    margin = 0.03
    for _n, pts_l, r, p, centres, radii, bounds in shapes:
        near = np.flatnonzero(np.all((centres + radii[:, None] >= lo - margin) & (centres - radii[:, None] <= hi + margin),
                                     axis=1))
        if not len(near):
            continue
        world = np.concatenate([pts_l[bounds[k]:bounds[k + 1]] for k in near]) @ r.T + p
        world = world[np.all((world >= lo - margin) & (world <= hi + margin), axis=1)]
        if not len(world):
            continue
        for c, rr, h in boxes[n_fixed:]:
            best_lead = min(best_lead, float(_box_gap(world, c, rr, h).min()) * 1000)
        for a, b, rad in caps:
            if best_lead <= 0.0:
                break
            best_lead = min(best_lead, float(_seg_gap(world, a, b, rad).min()) * 1000)
        best_all = min(best_all, best_lead)
        for c, rr, h in boxes[:n_fixed]:
            if best_all <= 0.0:
                break
            best_all = min(best_all, float(_box_gap(world, c, rr, h).min()) * 1000)
        if best_lead <= 0.0:
            return 0.0, 0.0
    return best_all, best_lead


def robot_gap_mm(shapes, objects) -> float:
    """Smallest distance (mm) from any robot point to the terminal box, the gland or any lead (plug or cable)."""
    return robot_gaps_mm(shapes, objects)[0]


# ---- the tester and the score --------------------------------------------------------------------------------------
TOUCH_MM = 2.0          # the robot has touched a plug or lead once a robot point is within this of one (mm); the
                        # supply stays off until then
RELEASE_MM = 20.0       # the supply comes on only with no robot point within this of the terminal box, gland or leads
STILL_MM = 1.0          # ... and no plug moved more than this over the last HOLD_STEPS steps (mm)
HOLD_STEPS = 20         # 1 s at 20 Hz: both conditions held this long before the supply comes on; success: full score
                        # held this long
HZ = 20

_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(step=0, recent=[], touched=False, clear_steps=0, powered=False, direction=0, tote=0.0, disc=0.0, report={},
                  score=0.0, full_steps=0)
    if environment is not None:
        _drive_targets(environment)


def _drive_targets(environment) -> None:
    """Hold the tote where the tester's belt has taken it, and spin the pointer disc while powered."""
    import torch

    art = environment.unwrapped.scene["drive"]
    names = list(art.joint_names)
    dev = art.data.joint_pos.torch.device if hasattr(art.data.joint_pos, "torch") else "cuda:0"
    it, idisc = names.index("tote_slide"), names.index("disc_pin")
    art.set_joint_position_target_index(target=torch.tensor([[_STATE["tote"]]], device=dev), joint_ids=[it])
    w = DISC_SPEED * _STATE["direction"] if _STATE["powered"] else 0.0
    art.set_joint_velocity_target_index(target=torch.tensor([[w]], device=dev), joint_ids=[idisc])


def objects_now(environment) -> dict:
    from yam_arena.scene import object_states

    return object_states(environment, [o.name for o in OBJECTS])


def evaluate(objects, shapes=None) -> dict:
    """Every check (no side effects): where each plug is, the robot's distance, the plugs' stillness over the last
    HOLD_STEPS steps, where the tote is, and the score."""
    shapes = robot_shapes() if shapes is None else shapes
    rep = measure(objects)
    gap, gap_lead = robot_gaps_mm(shapes, objects)
    rep["robot_mm"] = round(gap, 1) if math.isfinite(gap) else None
    rep["released"] = gap >= RELEASE_MM
    rep["touched"] = _STATE["touched"] or gap_lead <= TOUCH_MM     # the robot has touched a plug or lead by now
    recent = _STATE["recent"]
    rep["still"] = (len(recent) == HOLD_STEPS + 1 and
                    float(np.max(np.linalg.norm(recent[-1] - recent[0], axis=1))) * 1000 <= STILL_MM)
    tote = float(objects["drive"].sliders.get("tote", 0.0))   # m along the drive's +y
    rep["tote_mm"] = round(-tote * 1000, 1)             # + forward (-y in the drive frame)
    rep["past_forward_line"] = -tote >= FWD_MOVE - 1e-9
    rep["score"] = score_of(rep)
    return rep


def score_of(rep: dict) -> float:
    """0 unless the robot has touched a plug or lead at some point, is clear now and the plugs are still, and all three
    plugs are in three different sockets in an order that reverses the motor (odd relative to the start). Then 1.0 if all three are home and the tote has run wholly past
    the forward line; otherwise 0.5."""
    if not (rep["touched"] and rep["released"] and rep["still"] and rep["all_in"] and rep["odd"]):
        return 0.0
    return 1.0 if (rep["all_home"] and rep["past_forward_line"]) else 0.5


def _plug_centres(objects) -> np.ndarray:
    return np.array([plug_pose(objects[f"lead_{n}"])[0] for n in LEADS])


def _tick(environment) -> None:
    """After every control step: measure, run the tester (supply on after HOLD_STEPS clear steps; direction from the
    lead order), move the belt, score."""
    _STATE["step"] += 1
    cablekit.hold_shape(environment)          # the leads keep the shape they are left in (no spring-back)
    objects = objects_now(environment)
    recent = _STATE["recent"]
    recent.append(_plug_centres(objects))
    del recent[:-(HOLD_STEPS + 1)]
    shapes = robot_shapes()
    rep = evaluate(objects, shapes)
    _STATE["touched"] = rep["touched"]
    clear = rep["touched"] and rep["released"] and rep["still"] and rep["all_home"]
    _STATE["clear_steps"] = _STATE["clear_steps"] + 1 if clear else 0
    _STATE["powered"] = _STATE["clear_steps"] >= HOLD_STEPS
    if _STATE["powered"]:
        _STATE["direction"] = 1 if rep["odd"] else -1
        _STATE["tote"] = float(np.clip(_STATE["tote"] - _STATE["direction"] * TOTE_SPEED / HZ, -TOTE_STOP, TOTE_STOP))
    _drive_targets(environment)
    rep["powered"] = _STATE["powered"]
    rep["direction"] = _STATE["direction"] if _STATE["powered"] else 0
    _STATE["report"] = rep
    _STATE["score"] = rep["score"]
    _STATE["full_steps"] = _STATE["full_steps"] + 1 if rep["score"] >= 1.0 else 0


on_reset(_reset_state)
on_step(_tick)
_reset_state()


def success(objects) -> bool:
    """Full score held for 1 s."""
    return _STATE["full_steps"] >= HOLD_STEPS


def score(objects) -> float:
    """0 unless the robot has touched a plug or lead during the trial, no robot point is now within 20 mm of the terminal
    box, the gland or any lead, no plug moved more than 1 mm over the last second, and all three plugs are in three
    different sockets (lined up, tip at least 8 mm in) in an order that reverses the motor (an odd permutation of the
    starting order: two leads swapped). Then 1.0 if all three tips are at least 15 mm in and the tote has run wholly
    past the green (forward) line; otherwise 0.5."""
    return _STATE["score"]
