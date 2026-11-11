"""Check that the numbers in task.json, README.md and demo.py match task.py, and that the score follows its rule at every
boundary (socket depth marks, line-up, lead order parity, tote line, release zone).

    python check_task.py

Runs without the simulator (task.py is only imported). Exits 1 on any mismatch.
"""

import itertools
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import task as T  # noqa: E402

J = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()
DEMO = (HERE / "demo.py").read_text()
bad = []


def flat(text: str) -> str:
    return " ".join(text.split())


def need(text: str, where: str, phrase: str) -> None:
    if phrase not in flat(text):
        bad.append(f"{where}: missing {phrase!r}")


def g(v: float) -> str:
    return f"{round(v, 6):g}"


mm = lambda v: g(v * 1000)  # noqa: E731
cm = lambda v: g(v * 100)   # noqa: E731

# -- form ------------------------------------------------------------------------------------------------------------
for where, text in (("README", README), ("task.json", json.dumps(J))):
    for marker in ("{{", "TODO", "XX", "INSTR", "DREAMRESULT", "DEMORESULT", "VARIANTS", "GAPTEST", "RUNTIME",
                   "REALWORLD"):
        if marker in text:
            bad.append(f"{where}: unfilled marker {marker}")
for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):  # media: written by tools/upload_media.py at submission
    if not J.get(k):
        bad.append(f"task.json: missing {k}")
if len(J.get("fidelity", "")) > 500:          # enforced by the PR bot
    bad.append(f"task.json: fidelity is {len(J['fidelity'])} characters (max 500)")
if len(re.findall(r"[.!?](\s|$)", J.get("context", ""))) > 2:
    bad.append("task.json: context longer than 2 sentences")
if J["category"] not in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance", "Other"):
    bad.append("task.json: category")
if T.K != 0:
    bad.append("check must run with MOTOR_STATE unset (state 0)")

# the instruction, word for word, in task.py, task.json and the README's command
cmd = re.search(r'--instruction "([^"]+)"', README)
if not (cmd and cmd.group(1) == J["instruction"] == T.INSTRUCTION):
    bad.append("instruction differs between task.py, task.json and README")

# the run limit
steps = int(J["run_limit"].split()[0].replace(",", ""))
if f"--max-steps {steps}" not in README or f'"--max-steps", type=int, default={steps}' not in DEMO:
    bad.append("run limit differs between task.json, README and demo.py")
if J["run_limit"] != f"{steps:,} steps ({steps / 1200:g} minutes at 20 Hz)":
    bad.append("run limit text")
need(README, "README", f"(the run limit is {steps:,})")

# -- setup (state 0) -------------------------------------------------------------------------------------------------
if T.START != (0, 1, 2) or T.BOW != 1 or (T.FACE_X, T.FACE_Y, T.YAW) != (0.36, 0.0, 0.0):
    bad.append("state 0 must be brown U, black V, grey W, bow left, drive at x 36 cm, y 0, square")
lay = T.LEAD_OBJS[0].body_layout()
bow_y = float(lay[:, 1].max())
bow_x = float(lay[lay[:, 1] >= bow_y - 1e-4, 0].mean()) + T.SEG / 2   # middle of the bow's widest part
for o in T.LEAD_OBJS[1:]:
    if not np.allclose(o.body_layout(), lay):
        bad.append("the three leads' starting shapes differ at state 0")
setup = [
    f"Terminal box: blue-grey painted box {cm(T.BOX_D)} (x) x {cm(T.BOX_W)} (y) x {cm(T.BOX_H)} cm tall, front face at "
    f"x {cm(T.FACE_X)}",
    f"three sockets {mm(T.CAV_W)} mm wide x {mm(T.CAV_H)} mm tall x {mm(T.CAV_D)} mm deep, centred at y "
    f"+{cm(T.SOCKET_Y[0])} (U), {cm(T.SOCKET_Y[1])} (V) and {cm(T.SOCKET_Y[2])} (W)",
    f"blue-grey cylinder {cm(2 * T.MOTOR_R)} cm across and {cm(T.MOTOR_L)} cm long, axis along y, centred at "
    f"x {cm(T.FACE_X + T.MOTOR_X)}, y 0",
    f"dark grey block {cm(T.GEAR[1] - T.GEAR[0])} x {cm(T.GEAR[3] - T.GEAR[2])} x {cm(T.GEAR[4])} cm tall at "
    f"x {cm(T.FACE_X + T.GEAR[0])} to {cm(T.FACE_X + T.GEAR[1])}, y {cm(T.GEAR[3])} to {cm(T.GEAR[2])}",
    f"white pointer disc {cm(2 * T.DISC_R)} cm across",
    f"light grey bed {cm(T.BED_W)} (x) x {cm(T.BED_L)} (y) x {cm(T.BED_H)} cm tall centred at x {cm(T.FACE_X + T.BED_X)}, y 0",
    f"yellow guard plate {cm(T.GUARD_X1 - T.GUARD_X0)} x {cm(T.BED_L)} cm, {cm(T.GUARD_Z)} cm tall "
    f"(x {cm(T.FACE_X + T.GUARD_X0)} to {cm(T.FACE_X + T.GUARD_X1)})",
    f"a red line across it at y +{cm(T.LINE_Y)} and a green line at y -{cm(T.LINE_Y)}",
    f"blue box {cm(T.TOTE[0])} (x) x {cm(T.TOTE[1])} (y) x {cm(T.TOTE[2])} cm, {mm(T.TOTE_M)} g, on the belt centred at y 0",
    f"dark grey block {cm(T.GLAND_SIZE[0])} x {cm(T.GLAND_SIZE[1])} x {cm(T.GLAND_SIZE[2])} cm tall at "
    f"x {cm(T.GLAND_X - T.GLAND_SIZE[0])} to {cm(T.GLAND_X)}",
    f"three {mm(2 * T.R)} mm round PVC leads {cm(T.LEAD_LEN)} cm long",
    f"plug body {mm(T.PL)} x {mm(T.PW)} x {mm(T.PH)} mm (ABS, the lead's colour, {mm(T.PLUG_M)} g)",
    f"brown is pushed {mm(T.DEPTH0)} mm into U, black into V, grey into W",
    f"swings out to {bow_y * 100:.1f} cm left of its gland position about {round(bow_x * 100):d} cm from the gland",
]
for p in setup:
    need(J["setup"], "task.json setup", p)

# -- scoring text ----------------------------------------------------------------------------------------------------
ang = math.degrees(math.acos(0.9))
scoring = [f"tape marks {g(T.IN_MM)} mm and {g(T.HOME_MM)} mm from its tip",
           f"centre within {g(T.LINE_MM)} mm of the socket's centre line sideways",
           f"pointing into it within {math.floor(ang * 10) / 10:g} degrees",
           "its centre at most 5 mm above the opening's top",
           f"no part of either robot is within {g(T.RELEASE_MM)} mm of the terminal box, the gland or any lead",
           f"no plug moved more than {g(T.STILL_MM)} mm over the last second",
           f"wholly past the green (forward) line ({mm(T.FWD_MOVE)} mm forward)", "otherwise 0.5",
           f"at {g(T.TOTE_SPEED)} m/s", f"1.0 for {g(T.HOLD_STEPS / T.HZ)} s"]
for p in scoring:
    need(J["scoring"], "task.json scoring", p)
readme_scoring = [f"{g(T.IN_MM)} mm and {g(T.HOME_MM)} mm from its tip",
                  f"its centre within {g(T.LINE_MM)} mm of the socket's centre line",
                  f"pointing into the socket within {math.floor(ang * 10) / 10:g} degrees",
                  "its centre at most 5 mm above the opening's top",
                  f"no part of either robot is within {g(T.RELEASE_MM)} mm of the terminal box, the gland or any lead",
                  f"no plug moved more than {g(T.STILL_MM)} mm over the last second",
                  f"{mm(T.FWD_MOVE)} mm forward of where it started", "otherwise **0.5**",
                  f"the tote at {g(T.TOTE_SPEED)} m/s, up to {mm(T.TOTE_STOP)} mm either way",
                  f"once the score has been 1.0 for {g(T.HOLD_STEPS / T.HZ)} s"]
for p in readme_scoring:
    need(README, "README scoring", p)
if abs(T.FWD_MOVE - (T.LINE_Y + T.TOTE[1] / 2)) > 1e-12:
    bad.append("FWD_MOVE is not the distance for the tote's trailing face to pass the green line")

# -- fidelity --------------------------------------------------------------------------------------------------------
fid = ["not simulated electrically", "computed from the lead order", f"slider at {g(T.TOTE_SPEED)} m/s", "fully plastic, a simplification: no restoring force",
       f"{T.LEAD_OBJS[0].segments} rigid {cm(T.SEG)} cm segments",
       f"sockets {mm(T.CAV_W)} x {mm(T.CAV_H)} mm for a {mm(T.PW)} x {mm(T.PH)} mm plug"]
for p in fid:
    need(J["fidelity"], "task.json fidelity", p)
for p in [f"chains of {T.LEAD_OBJS[0].segments} rigid {cm(T.SEG)} cm segments with bending springs "
          f"({g(T.BEND_K)} N m/rad per joint", f"Sockets are {mm(T.CAV_W)} x {mm(T.CAV_H)} mm and {mm(T.CAV_D)} mm deep",
          f"{g((T.CAV_W - T.PW) / 2 * 1000)} mm of play each side and {g((T.CAV_H - T.PH) * 1000)} mm above it"]:
    need(README, "README fidelity", p)
slack = T.LEAD_LEN - (T.FACE_X - (T.PL - T.DEPTH0) - (T.GLAND_X + 0.001))
need(README, "README fidelity", f"each has {round(slack * 100):d} cm of slack")

# -- the score at its boundaries (fake lead states, no simulator) ----------------------------------------------------


@dataclass
class _Lead:
    points: np.ndarray
    end_plug: np.ndarray


def _lead_at(socket: int, depth_mm: float, side_mm: float = 0.0, yaw_deg: float = 0.0) -> _Lead:
    """A plug at ``socket`` (drive frame): centre ``side_mm`` off the socket's centre line, turned ``yaw_deg``, its tip
    ``depth_mm`` inside the face."""
    a = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)), 0.0])
    centre = np.array([depth_mm / 1000 - a[0] * T.PL / 2, T.SOCKET_Y[socket] + side_mm / 1000, T.PH / 2 + 0.0005])
    seg = centre - a * (T.PL / 2 + T.SEG / 2)
    return _Lead(points=np.array([T.to_world(seg - a * T.SEG), T.to_world(seg)]), end_plug=T.to_world(centre))


def _objs(where) -> dict:
    return {f"lead_{n}": _lead_at(*w) for n, w in zip(T.LEADS, where)}


def _rep(where, released=True, still=True, tote_mm=0.0, touched=True):
    rep = T.measure(_objs(where))
    rep.update(released=released, still=still, touched=touched,
               past_forward_line=tote_mm / 1000 >= T.FWD_MOVE - 1e-9)
    return rep, T.score_of(rep)


# every order of the three leads: odd (one swap) -> reversing, even (start, cyclic) -> not
for perm in itertools.permutations(range(3)):
    rep, _ = _rep([(perm[i], 24.0) for i in range(3)], tote_mm=130)
    odd = sum(1 for x in range(3) for y in range(x + 1, 3) if perm[x] > perm[y]) % 2 == 1
    if rep["odd"] != odd or (_rep([(perm[i], 24.0) for i in range(3)], tote_mm=130)[1] != (1.0 if odd else 0.0)):
        bad.append(f"order {perm}: odd {rep['odd']}, want {odd}")
swap = [(2, 24.0), (1, 24.0), (0, 24.0)]       # brown W, black V, grey U
for where, tote, kw, want in [
        (swap, 130, {}, 1.0), (swap, T.FWD_MOVE * 1000, {}, 1.0), (swap, T.FWD_MOVE * 1000 - 0.1, {}, 0.5),
        (swap, 130, {"released": False}, 0.0), (swap, 130, {"still": False}, 0.0), (swap, 130, {"touched": False}, 0.0),
        ([(2, T.HOME_MM), (1, 24.0), (0, 24.0)], 130, {}, 1.0),
        ([(2, T.HOME_MM - 0.1), (1, 24.0), (0, 24.0)], 130, {}, 0.5),
        ([(2, T.IN_MM), (1, 24.0), (0, 24.0)], 0, {}, 0.5),
        ([(2, T.IN_MM - 0.1), (1, 24.0), (0, 24.0)], 0, {}, 0.0),
        ([(2, 24.0, T.LINE_MM - 0.1), (1, 24.0), (0, 24.0)], 130, {}, 1.0),
        ([(2, 24.0, T.LINE_MM + 0.1), (1, 24.0), (0, 24.0)], 130, {}, 0.0),
        ([(2, 24.0, 0.0, 25.0), (1, 24.0), (0, 24.0)], 130, {}, 1.0),
        ([(2, 24.0, 0.0, 26.0), (1, 24.0), (0, 24.0)], 130, {}, 0.0),
        ([(1, 24.0), (2, 24.0), (0, 24.0)], 130, {}, 0.0),            # cyclic shift
        ([(0, 24.0), (1, 24.0), (2, 24.0)], 130, {}, 0.0)]:          # unchanged
    sc = _rep(where, tote_mm=tote, **kw)[1]
    if sc != want:
        bad.append(f"score {where} tote {tote} {kw}: {sc}, want {want}")

# -- the release zone: a robot point on any scored box or lead reads 0 mm; one 0.5 m away reads more than RELEASE_MM --
objs = _objs([(0, 24.0), (1, 24.0), (2, 24.0)])
for n in T.LEADS:   # give the fake leads a full polyline for the capsules
    st = objs[f"lead_{n}"]
    st.polyline = (lambda s=st: np.vstack([s.points, s.end_plug]))


def _shape(pts):
    pts = np.atleast_2d(np.asarray(pts, float))
    return [("fake", pts, np.eye(3), np.zeros(3), pts, np.zeros(len(pts)), np.arange(len(pts) + 1))]


worst = 0.0
for c, r, h in T.scored_boxes(objs):
    for ax in range(3):
        for sgn in (-1, 1):
            worst = max(worst, T.robot_gap_mm(_shape(c + r @ (np.eye(3)[ax] * h[ax] * sgn)), objs))
for a, b, rad in T.scored_capsules(objs):
    m = (a + b) / 2
    worst = max(worst, T.robot_gap_mm(_shape(m + np.array([0, 0, rad])), objs))
if worst > 1e-6:
    bad.append(f"a robot point on a scored part reads {worst:.3f} mm, not 0")
box_top = T.to_world((T.BOX_D / 2, 0.0, T.BOX_H))
g_all, g_lead = T.robot_gaps_mm(_shape(box_top), objs)
if not (g_all < 1e-6 and g_lead > T.TOUCH_MM):
    bad.append("a robot point on the terminal box's top counts as touching a lead")
plug_top = T.plug_pose(objs["lead_black"])[0] + [0, 0, T.PH / 2]
if T.robot_gaps_mm(_shape(plug_top), objs)[1] > 1e-6:
    bad.append("a robot point on a plug's top does not count as touching a lead")
if not T.robot_gap_mm(_shape([T.FACE_X - 0.5, 0.0, T.TOP + 0.3]), objs) > T.RELEASE_MM:
    bad.append("a point 0.5 m away reads inside the release zone")

print("\n".join(bad) if bad else "OK: task.json, README and demo.py match task.py")
sys.exit(1 if bad else 0)
