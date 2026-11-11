"""Check that the numbers in task.json, README.md and demo.py match task.py, and that placement_score() follows the
README's score table at every boundary.

    python check_task.py

Runs without the simulator (task.py is only imported). Exits 1 on any mismatch.
"""

import itertools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import task as T  # noqa: E402
import demo as D  # noqa: E402

J = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()
DEMO = (HERE / "demo.py").read_text()
bad = []


def need(text: str, where: str, phrase: str) -> None:
    if phrase not in " ".join(text.split()):
        bad.append(f"{where}: missing {phrase!r}")


def g(v: float) -> str:
    return f"{v:g}"


# the instruction, word for word, in task.py, task.json and the README's command
cmd = re.search(r'--instruction "([^"]+)"', README)
if not (cmd and cmd.group(1) == J["instruction"] == T.INSTRUCTION):
    bad.append("instruction differs between task.py, task.json and README")
for word in ("line up", "align"):
    if word in T.INSTRUCTION.lower():
        bad.append(f"instruction uses {word!r}")

# no placeholders or to-do notes left in the submitted text
for where, text in (("README", README), ("task.json", json.dumps(J))):
    for hit in re.findall(r"\b(?:TODO|TBD|FIXME|XXX|[A-Z]{3,}_[A-Z_]{3,})\b", text):
        if hit not in ("DRIVE_STATE", "DEMO_VARIANT", "DEMO_DEBUG"):        # the demo's environment variables
            bad.append(f"{where}: placeholder {hit!r}")

# the run limit
steps = int(J["run_limit"].split()[0].replace(",", ""))
if f"--max-steps {steps}" not in README or f'"--max-steps", type=int, default={steps}' not in DEMO:
    bad.append("run limit differs between task.json, README and demo.py")
if J["run_limit"] != f"{steps:,} steps ({steps / 1200:g} minute{'' if steps == 1200 else 's'} at 20 Hz)":
    bad.append("run limit text")
for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
    if not J.get(key):
        bad.append(f"task.json: {key} missing")
if not re.fullmatch(r"Dream-1 run: [01]\.\d+; demo: 1\.0", J["scores"]):
    bad.append("task.json: scores")
if "media" in J:
    bad.append("task.json: media is filled in by the upload script")
if len(J["fidelity"]) > 500:                  # the PR bot rejects a longer fidelity field
    bad.append(f"task.json: fidelity is {len(J['fidelity'])} characters (max 500)")
if J["category"] not in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance",
                         "Other"):
    bad.append("task.json: category")

# geometry derived from the parts as built
assert T.K == 0, "run with DRIVE_STATE unset (the task's layout)"
sleeve_corners = np.array([c + r @ (np.array(s) * h) for c, h, r in T._SLEEVE_BOXES
                           for s in itertools.product((-1, 1), repeat=3)]) * 1000
sleeve_len = sleeve_corners[:, 2].max() - sleeve_corners[:, 2].min()
assert abs(sleeve_corners[:, 2].min()) < 1e-6 and abs(sleeve_len - T.S_EAR_Z[1]) < 1e-6
# the play: a land's inner corner (radius hypot(LAND_R[0], LAND_W/2)) meeting a spline's flat side (KEY_W/2 off its line)
corner_r = math.hypot(T.LAND_R[0], T.LAND_W / 2)
play = 360 / T.N_SPLINE / 2 - math.degrees(math.atan(T.LAND_W / 2 / T.LAND_R[0])) - \
    math.degrees(math.asin(T.KEY_W / 2 / corner_r))
ear_rel = math.degrees(T.SLEEVE_YAW0 + math.pi / 2 - T.PHI1)
cross_long = 2 * T.EAR_R[1]
vol_cm3 = (math.pi / 4 * ((2 * T.RING_R[1]) ** 2 - (2 * T.RING_R[0]) ** 2) * T.SLEEVE_L
           - T.BEVEL ** 2 / 2 * 2 * math.pi * (T.RING_R[0] + T.BEVEL / 3)
           + T.N_SPLINE * (T.RING_R[0] - T.LAND_R[0]) * T.LAND_W * (T.SLEEVE_L - T.LAND_Z0)
           + 2 * (T.EAR_R[1] - T.EAR_R[0]) * T.EAR_W * (T.S_EAR_Z[1] - T.S_EAR_Z[0])) / 1000
setup = [f"axis at x {g(T.FX * 100)}, y {g(T.FY * 100)}", "base plate 80 x 80 x " + g(T.BASE_H) + " mm",
         f"each {g(T.EAR_R[1] - T.EAR_R[0])} mm thick, {g(T.EAR_W)} mm wide and {g(T.EAR_Z[1] - T.EAR_Z[0])} mm tall "
         f"({g(T.EAR_Z[0])} to {g(T.EAR_Z[1])} mm up)",
         f"inner faces {g(T.EAR_R[0])} mm and outer faces {g(T.EAR_R[1])} mm from the axis",
         f"a {g(2 * T.CROSS_R)} x {g(2 * T.CROSS_R)} mm bar {g(cross_long)} mm long", "a 32 mm bar across it",
         f"centred {g(T.CROSS_Z)} mm up", f"tube, {g(2 * T.TUBE_R)} mm across, from {g(T.TUBE_Z[0])} to "
         f"{g(T.TUBE_Z[1])} mm up", f"collar {g(2 * T.COLLAR_R)} mm across, from {g(T.COLLAR_Z[0])} to "
         f"{g(T.COLLAR_Z[1])} mm up", f"a {g(2 * T.CORE_R)} mm core from {g(T.SPLINE_Z[0])} to {g(T.SPLINE_Z[1])} mm up",
         f"{'six' if T.N_SPLINE == 6 else T.N_SPLINE} flat-topped splines",
         f"each {g(T.KEY_W)} mm wide and standing {g(T.KEY_R[1] - T.KEY_R[0])} mm proud of the core "
         f"({g(2 * T.KEY_R[1])} mm across the tips)", f"every {g(360 / T.N_SPLINE)} degrees",
         f"PETG, {g(T.SLEEVE_MASS * 1000)} g", f"a sleeve {g(T.SLEEVE_L)} mm long, {g(2 * T.RING_R[1])} mm across "
         f"outside and {g(2 * T.RING_R[0])} mm across its bore", f"lands {g(T.LAND_W)} mm wide reaching to "
         f"{g(T.LAND_R[0])} mm from its axis ({g(2 * T.LAND_R[0])} mm across)",
         f"a {g(T.BEVEL)} mm 45-degree bevel", f"lands start {g(T.LAND_Z0)} mm above the bottom",
         f"each {g(T.EAR_R[1] - T.EAR_R[0])} mm thick, {g(T.EAR_W)} mm wide and {g(T.S_EAR_Z[1] - T.S_EAR_Z[0])} mm tall, "
         f"their outer faces {g(2 * T.EAR_R[1])} mm apart", f"at x {g(T.SLEEVE.pos[0] * 100)}, y {g(T.SLEEVE.pos[1] * 100)}",
         f"ears pointing {ear_rel:.0f} degrees from the far yoke's ear line",
         f"any of {'six' if T.N_SPLINE == 6 else T.N_SPLINE} turns, {g(360 / T.N_SPLINE)} degrees apart"]
scoring = [f"bottom within {g(T.SEAT_MM)} mm of the collar's top", f"axis within {g(T.AXIS_MM)} mm of the shaft's",
           f"within {g(T.TILT_MAX)} degrees of vertical", f"at least {g(T.ENGAGED_MM)} mm of the sleeve below the top of",
           f"Score 1.0 if the phase is {g(T.PHASE_TOL)} degrees or less", f"0.5 if the phase is {g(T.PHASE_TOL)} degrees",
           f"phase more than {g(T.PHASE_TOL)} degrees", f"within {g(T.RELEASE_MM)} mm of the slip yoke",
           f"less than {g(T.STILL_MM)} mm over the last second", f"1.0 for {g(T.HOLD_STEPS / 20)} s"]
fidelity = [f"{'six' if T.N_SPLINE == 6 else T.N_SPLINE}, {g(360 / T.N_SPLINE)} degrees apart",
            f"every wrong position is {g(360 / T.N_SPLINE)} degrees out", f"the {g(T.KEY_W)} mm splines",
            f"a {g(T.BEVEL)} mm 45-degree lead-in bevel", f"lands start {g(T.LAND_Z0)} mm up",
            f"the play is {play:.1f} degrees each way (calculated)",
            f"Friction {g(T.FIXTURE.friction)} on both parts", f"weighs {g(T.SLEEVE_MASS * 1000)} g"]
if T.FIXTURE.friction != T.SLEEVE.friction:
    bad.append("friction differs between the parts")
for p in setup:
    need(J["setup"], "task.json setup", p)
for p in scoring:
    need(J["scoring"], "task.json scoring", p)
for p in fidelity:
    need(J["fidelity"], "task.json fidelity", p)
need(J["reset_notes"], "task.json reset_notes", f"pointing {ear_rel:.0f} degrees")

readme = [f"a {g(T.SPLINE_Z[1] - T.SPLINE_Z[0])} mm splined shaft with six splines",
          f"orange {g(T.SLEEVE_L)} mm splined sleeve", f"any of six turns, {g(360 / T.N_SPLINE)} degrees apart",
          f"about {round((T.COLLAR_Z[1] + sum(T.S_EAR_Z) / 2 - sum(T.EAR_Z) / 2) / 10):g} cm away",
          f"Phase {g(T.PHASE_TOL)} degrees or less, and seated | 1.0",
          f"Phase {g(T.PHASE_TOL)} degrees or less, on the shaft but not seated | 0.5",
          f"Phase more than {g(T.PHASE_TOL)} degrees",
          f"within {g(T.RELEASE_MM)} mm of the slip yoke, or while it moved {g(T.STILL_MM)} mm or more in the last second",
          f"bottom within {g(T.SEAT_MM)} mm of the collar's top", f"a {g(T.SEAT_MM)} mm feeler",
          f"within {g(T.AXIS_MM)} mm of the shaft's and within {g(T.TILT_MAX)} degrees of vertical",
          f"at least {g(T.ENGAGED_MM)} mm of the sleeve below the top of the splines",
          f"every {g(T.SAMPLE_MM)} mm, so any surface within {T.RELEASE_MM - 1.25:.2f} mm is always caught",
          f"once the score has been 1.0 for {g(T.HOLD_STEPS / 20)} s", f"the {g(T.PHASE_TOL)}-degree window",
          *fidelity[3:], "7.38 degrees at 60 degrees out of phase and 8.21 degrees at 90", "0.00, 0.76 and 0.88",
          f"weighs about {g(T.SLEEVE_MASS * 1000)} g, calculated from its volume of {vol_cm3:.1f} cm3",
          f"x {g(T.FX * 100)}, y {g(T.FY * 100)} cm", f"x {g(T.SLEEVE.pos[0] * 100)}, y {g(T.SLEEVE.pos[1] * 100)} cm",
          f"pointing {ear_rel:.0f} degrees from the far", f"ears at {ear_rel:.0f} degrees",
          f"the slip yoke weighs {g(T.SLEEVE_MASS * 1000)} g",
          f"{g(D.MIS_DEG)} degrees off the in-phase", f"{g(D.ENTRY_MM)} mm above the splines' top",
          f"{g(D.PRESS_MM)} mm below the collar's top", f"{g(D.GRIP_Z * 1000)} mm above the sleeve's bottom"]
readme += [f"between {play:.1f} and {360 / T.N_SPLINE - play:.1f} degrees (the play; calculated)",
           f"within {play:.1f} degrees of one or the other", f"inner corner, {corner_r:.2f} mm from the",
           f"({g(T.EAR_R[1] - T.EAR_R[0])} mm thick, {g(T.EAR_R[0])} to {g(T.EAR_R[1])} mm from the axis, {g(T.EAR_W)} mm",
           f"slip yoke's start {ear_rel:.0f} degrees from them"]
for p in readme:
    need(README, "README", p)
if abs(vol_cm3 * 1.27 - T.SLEEVE_MASS * 1000) > 1.0:
    bad.append(f"slip yoke mass {T.SLEEVE_MASS * 1000} g vs solid PETG {vol_cm3 * 1.27:.1f} g")

# the Cardan figures quoted in the README (calculated, not simulated)
th = np.linspace(0, 2 * np.pi, 200001)
for beta, quoted in ((30, (0.00, 7.38, 8.21)), (10, (0.00, 0.76, 0.88))):
    b = math.radians(beta)
    t2 = np.arctan2(np.sin(th), np.cos(th) * np.cos(b))           # first joint: tan(t2) = tan(t1) / cos(b)
    for ph, q in zip((0, 60, 90), quoted):
        a = t2 - math.radians(ph)
        t3 = math.radians(ph) + np.arctan2(np.sin(a) * np.cos(b), np.cos(a))   # second joint, its yoke turned ph
        err = float(np.abs(np.degrees(np.angle(np.exp(1j * (t3 - th))))).max())
        if abs(round(err, 2) - q) > 1e-9:
            bad.append(f"Cardan error beta {beta} phase {ph}: {err:.2f}, README {q}")

# the score table: every boundary and a point either side of it
eps = 1e-6
rows = [(lambda e: e["phase_deg"] <= T.PHASE_TOL and e["seated"], 1.0),
        (lambda e: e["phase_deg"] <= T.PHASE_TOL and e["on_shaft"] and not e["seated"], 0.5),
        (lambda e: e["phase_deg"] > T.PHASE_TOL or not e["on_shaft"], 0.0)]
n = 0
for phase in sorted({T.PHASE_TOL - eps, T.PHASE_TOL, T.PHASE_TOL + eps} | {x / 2 for x in range(0, 181)}):
    for height in sorted({T.SEAT_MM - eps, T.SEAT_MM, T.SEAT_MM + eps, -0.5, 10.0, 26.0, 27.0, 28.0, 42.0}):
        for off, tilt in ((0.0, 0.0), (T.AXIS_MM + eps, 0.0), (0.0, T.TILT_MAX + eps), (T.AXIS_MM, T.TILT_MAX)):
            engaged = T.SPLINE_Z[1] - T.COLLAR_Z[1] - height
            ok = off <= T.AXIS_MM and tilt <= T.TILT_MAX
            e = {"phase_deg": phase, "seated": height <= T.SEAT_MM and ok, "on_shaft": engaged >= T.ENGAGED_MM and ok}
            hits = [v for f, v in rows if f(e)]
            n += 1
            if len(hits) != 1 or T.placement_score(e) != hits[0]:
                bad.append(f"score table: {e} -> {T.placement_score(e)}, rows {hits}")

print(f"{n} score-table points checked; {len(setup) + len(scoring) + len(fidelity) + 1} task.json phrases; "
      f"{len(readme)} README phrases")
for b in bad:
    print("MISMATCH", b)
print("OK" if not bad else f"{len(bad)} mismatches")
sys.exit(1 if bad else 0)
