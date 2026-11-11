"""Check that the numbers in task.json, README.md and demo.py match task.py, and that sag_score() follows the score
bands at every boundary.

    python check_task.py

Runs without the simulator (task.py is only imported). Exits 1 on any mismatch.
"""

import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
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


for where, text in (("README", README), ("task.json", json.dumps(J))):
    if "{{" in text or "TODO" in text:
        bad.append(f"{where}: unfilled marker")
for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
    if not J.get(k):
        bad.append(f"task.json: missing {k}")
if len(J.get("fidelity", "")) > 500:          # enforced by the PR bot (PR #1563 was bounced for 776 chars)
    bad.append(f"task.json: fidelity is {len(J['fidelity'])} characters (max 500)")
if J["category"] not in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance", "Other"):
    bad.append("task.json: category")

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

# the task's layout (state 0) and its parts
if T.K != 0:
    bad.append("check must run with BRACE_STATE unset (state 0)")
mm = lambda v: g(v * 1000)  # noqa: E731
cm = lambda v: g(v * 100)   # noqa: E731
stile, rail_len = T.STILE, T.W + T.STILE[1]
bottom_sq = ((T.Z_BOT + T.Z_TOP) / 2 - stile[2] / 2)
setup = [f"two stiles (green-painted pine, {mm(stile[0])} x {mm(stile[1])} x {mm(stile[2])} mm",
         f"two rails (green-painted pine, {mm(T.RAIL[0])} x {mm(T.RAIL[1])} x {mm(rail_len)} mm)",
         f"The pins are {mm(T.W)} mm apart along the rails (at y +{cm(T.Y_HINGE)} and y {cm(T.Y_LATCH)})",
         f"at {mm(T.Z_BOT)} mm and {mm(T.Z_TOP)} mm above the table, all in the plane x {cm(T.FX)}",
         f"its bottom {mm(bottom_sq)} mm above the table",
         f"an {mm(T.WEIGHT_M)} g steel block", f"a {mm(T.HANDLE[0])} x {mm(T.HANDLE[1])} x {mm(T.HANDLE[2])} mm wooden handle",
         f"{mm(T.HANDLE_Z)} mm above the table", f"stop block (30 x 30 x {mm(T.STOP_TOP)} mm)",
         f"{mm(T.SAG)} mm lower than when the frame is square",
         f"two pine plates ({mm(T.SEAT_DX)} x {mm(T.SEAT_L)} x {mm(T.SEAT_T)} mm, at 90 degrees",
         f"standing {mm(T.SEAT_DX)} mm out from the frame's front face",
         f"a pine dowel {mm(2 * T.R_SHAFT)} mm across with a {mm(2 * T.R_END)} mm ball on each end",
         f"{T.L_BRACE * 1000:.1f} mm between the balls' centres ({(T.L_BRACE + 2 * T.R_END) * 1000:.1f} mm overall), "
         f"{mm(T.BRACE_M)} g", f"centre at x {cm(T.BSTART[0])}, y {cm(T.BSTART[1])}"]
for p in setup:
    need(J["setup"], "task.json setup", p)
if not (T.SGN == 1 and T.Y_HINGE > 0):
    bad.append("state 0 must have the hinge on the left (+y)")

bands = T.SAG_BANDS
scoring = [f"each ball end within {g(T.SEATED_MM)} mm of its notch's seat point",
           f"no part of either robot is within {g(T.RELEASE_MM)} mm of the brace",
           f"no more than {g(T.STILL_MM)} mm over the last second",
           "; ".join(f"within {g(lim)} mm {sc:.1f}".replace(".0;", ".0;") for lim, sc in bands).replace(" 0.5", " 0.5")
           .replace(" 0.2", " 0.2") + "; further 0",
           f"1.0 for {g(T.HOLD_STEPS / 20)} s"]
for p in scoring:
    need(J["scoring"], "task.json scoring", p)
readme_scoring = [f"within {g(T.SEATED_MM)} mm of its notch's seat point",
                  f"no part of either robot is within {g(T.RELEASE_MM)} mm",
                  f"moved no more than {g(T.STILL_MM)} mm over the last second"] + \
                 [f"within {g(lim)} mm: **{g(sc) if sc != 1 else '1.0'}**" for lim, sc in bands]
for p in readme_scoring:
    need(README, "README scoring", p)
need(J["instruction"], "instruction", f"no more than {g(bands[0][0])} mm from level")

# fidelity numbers
fid = [f"less 0.8 mm" if abs(T.CLEAR - 0.0008) < 1e-12 else "CLEAR", f"Friction {g(T.GATE.friction)} (gate, brace) and "
       f"{g(T.STOP.friction)} (rubber stop)", f"the {mm(T.WEIGHT_M)} g weight"]
for p in fid:
    need(J["fidelity"], "task.json fidelity", p)
if T.BRACE.friction != T.GATE.friction:
    bad.append("brace friction differs from the gate's")
need(README, "README", f"{T.L_BRACE * 1000:.1f} mm between the balls' centres")
need(README, "README", f"sagged. A V-notch seat")  # intro wording present
need(README, "README", f"pulled that side down {mm(T.SAG)} mm onto a rubber stop")
need(README, "README", f"less {g(T.CLEAR * 1000)} mm clearance")

# the diagonals' length changes quoted in the README (at the task's sag)
a, b = [__import__("numpy").array(T.CORNERS[k]) for k in ("bh", "tl")]
c, d = [__import__("numpy").array(T.CORNERS[k]) for k in ("th", "bl")]
np = __import__("numpy")
comp = (np.linalg.norm((b - [0, T.SAG]) - a) - np.linalg.norm(b - a)) * 1000
tens = (np.linalg.norm((d - [0, T.SAG]) - c) - np.linalg.norm(d - c)) * 1000
need(README, "README", f"(by {-comp:.1f} mm at this gate's {mm(T.SAG)} mm sag)")
need(README, "README", f"gets longer (by {tens:.1f} mm)")

# score bands at their boundaries
for sag, want in ((0.0, 1.0), (3.0, 1.0), (-3.0, 1.0), (3.01, 0.5), (6.0, 0.5), (6.01, 0.2), (9.0, 0.2), (9.01, 0.0),
                  (-9.01, 0.0), (14.0, 0.0)):
    if T.sag_score(sag) != want:
        bad.append(f"sag_score({sag}) = {T.sag_score(sag)}, want {want}")

print("\n".join(bad) if bad else "OK: task.json, README and demo.py match task.py")
sys.exit(1 if bad else 0)
