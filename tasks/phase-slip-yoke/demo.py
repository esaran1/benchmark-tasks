"""Scripted demo for the slip-yoke phasing task, run in the yam_arena embodiment (closed loop on the measured slip-yoke
pose).

    python demo.py task.py OUT_DIR [--max-steps 1200] [--no-record] [--variants correct,nearest,...]

The left arm grips the slip yoke from above across its sleeve, between its ears, and lifts it. It picks the in-phase
spline position nearer the sleeve's starting turn (ears on the far yoke's ear line, either way round), turns its wrist
so the ears will point that way, and carries the sleeve over the shaft. Above the shaft it reads the sleeve's pose
three times and corrects its hand's position and turn from it. It pushes the sleeve slowly onto the splines and down to
the collar; if the sleeve stops on the way it lifts 2 mm, re-centres it from its measured pose and pushes again (twice
at most, then it leaves it where it stopped). It lets go, lifts off and goes back to where it started.

Environment: DEMO_VARIANT selects a failure variant (README, Scripted demo):
  nearest      the sleeve turned to the spline position nearest its starting turn (60 or 120 degrees from the far
               yoke's ears: out of phase in every starting state), pushed down and released
  other_wrong  the other out-of-phase position (120 or 60 degrees)
  misaligned   turned 8 degrees off the in-phase position (inside the 10-degree phase window but between spline
               positions), lowered onto the shaft and pressed, released
  hold         the demo, but the arm keeps hold of the sleeve to the end
  none         nothing moved
DRIVE_STATE=k (0-9) runs the gap test's other starting states. DEMO_DEBUG=1 prints the score report every 10 steps.
--variants runs several variants one after another in one process (the scene is reset between them) and writes
OUT_DIR/<variant>/result.json for each; used for the gap test.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
GRIP_Z = 0.028          # the tool point this far above the sleeve's bottom while gripping (m)
CARRY_Z = 0.155         # tool point height above the table while carrying (m)
ENTRY_MM = 3.0          # the sleeve's bottom this far above the splines' top before the slow push
PRESS_MM = 1.0          # the push aims the sleeve's bottom this far below the collar's top
MIS_DEG = 8.0           # misaligned: degrees off the in-phase position
RECENTRE = 2            # times the push re-centres a sleeve that has stopped above the collar
YAW_RANGE = (-150.0, 60.0)   # left-arm wrist yaw (deg) that its IK reaches well


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik
        from yam_arena.constants import READY
        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        if pe > 0.001:            # retry from the ready pose if warm-starting got stuck
            q2, pe2, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(READY, float), base_pos=self.base,
                                  iters=200)
            if pe2 < pe:
                q, pe = q2, pe2
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


class Ctx:
    def __init__(self, emb):
        from yam_arena import station
        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):
        from yam_arena.scene import object_states
        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.left.cmd, self.right.cmd))


def _wrap(a: float, period: float = 360.0) -> float:
    """``a`` wrapped to (-period/2, period/2]."""
    return -((-a + period / 2) % period - period / 2)


def ear_deg(st) -> float:
    """The sleeve's ear direction about the vertical (deg from +x)."""
    e = st.rotation()[:, 1]
    return math.degrees(math.atan2(e[1], e[0]))


def target_rel(T, variant: str, rel0: float) -> float:
    """Where the variant turns the ears, in degrees from the far yoke's ear line; rel0 is where they start."""
    pitch = 360.0 / T.N_SPLINE
    phased = min((0.0, 180.0), key=lambda p: abs(_wrap(rel0 - p, 180.0)))
    nearest = round(rel0 / pitch) * pitch
    if variant == "nearest":
        return nearest
    if variant == "other_wrong":
        return 180.0 - nearest
    if variant == "misaligned":
        return phased + MIS_DEG
    return phased


def plan_yaws(rel0: float, rel1: float, ear0: float) -> tuple[float, float]:
    """Wrist yaws (deg) at pick-up and at the shaft. The grip closes across the sleeve, so the pick-up yaw is the ear
    direction mod 180 (the gripper is the same turned half a turn). The wrist then turns the sleeve by rel1 - rel0,
    give or take half turns (the ears' symmetry). Both yaws inside YAW_RANGE, the smallest wrist turn first, then the
    most central."""
    turn = _wrap(rel1 - rel0, 180.0)
    mid = (YAW_RANGE[0] + YAW_RANGE[1]) / 2
    best = None
    for n in range(-3, 4):
        for m in range(-2, 3):
            g0 = ear0 + 180.0 * n
            g1 = g0 + turn + 180.0 * m
            if YAW_RANGE[0] <= min(g0, g1) and max(g0, g1) <= YAW_RANGE[1]:
                cost = (round(abs(g1 - g0), 6), abs((g0 + g1) / 2 - mid))
                if best is None or cost < best[0]:
                    best = (cost, g0, g1)
    if best is None:
        raise RuntimeError(f"no wrist yaw pair for ears at {ear0:.1f} turning {turn:.1f}")
    return best[1], best[2]


def script(T, ctx, variant: str, log: dict):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation
    if variant == "none":
        yield from wait(3.0)
        return
    arm = ctx.left
    name = T.SLEEVE.name
    st = ctx.states()[name]
    ear0 = ear_deg(st)
    rel0 = _wrap(ear0 - math.degrees(T.PHI1))
    rel1 = target_rel(T, variant, rel0)
    g0, g1 = plan_yaws(rel0, rel1, ear0)
    log.update(rel0=round(rel0, 2), rel_target=rel1, yaw_pick=round(g0, 1), yaw_place=round(g1, 1))
    rot0, rot1 = grip_rotation(math.radians(g0)), grip_rotation(math.radians(g1))
    b = np.asarray(st.pos)                                      # the sleeve's bottom centre
    g = b + [0, 0, GRIP_Z]
    yield from arm.move(g + [0, 0, 0.08], rot0, 2.0, grip=1.0)
    yield from arm.move(g, rot0, 1.2)
    yield from arm.set_grip(0.0, 0.8)
    yield from arm.move(np.array([g[0], g[1], T.TOP + CARRY_Z]), rot0, 1.2)
    over = np.array([T.FX, T.FY, T.TOP + CARRY_Z])
    yield from arm.move(over, rot1, 2.5)
    want_ear = math.degrees(T.PHI1) + rel1
    yaw = g1

    def corrected(bottom_mm: float, settle: float, duration: float = 0.8):
        """Wait, read the sleeve's pose, and move so its bottom centre is over the shaft's axis at bottom_mm above
        the table with its ears at want_ear (mod 180)."""
        nonlocal yaw
        yield from wait(settle)
        s = ctx.states()[name]
        c = np.asarray(s.pos)
        yaw += _wrap(want_ear - ear_deg(s), 180.0)
        xy = arm.pos[:2] + (np.array([T.FX, T.FY]) - c[:2])
        z = arm.pos[2] + (T.TOP + bottom_mm / 1000 - c[2])
        yield from arm.move(np.array([xy[0], xy[1], z]), grip_rotation(math.radians(yaw)), duration)

    entry = T.SPLINE_Z[1] + ENTRY_MM
    yield from corrected(entry + 10, 0.6, 1.5)
    for _ in range(3):
        yield from corrected(entry, 0.6)
    s = ctx.states()[name]
    log["entry"] = {"axis_off_mm": round(math.hypot(s.pos[0] - T.FX, s.pos[1] - T.FY) * 1000, 2),
                    "ear_err_deg": round(_wrap(want_ear - ear_deg(s), 180.0), 2)}
    # slow push: about 8 mm/s, the sleeve's bottom aimed PRESS_MM below the collar's top. If the sleeve stops on the
    # way (its bore's edge or a land on a spline's top), lift 2 mm, re-centre it from its measured pose and push again,
    # up to RECENTRE times; if it is still stuck, leave it there (back off 2 mm).
    z_end = arm.pos[2] - (entry - (T.COLLAR_Z[1] - PRESS_MM)) / 1000
    hist, tries = [], 0
    while arm.pos[2] > z_end:
        arm._solve(arm.pos + [0, 0, -max(0.0, min(0.0004, arm.pos[2] - z_end))], arm.rot)
        yield
        hist.append(float(ctx.states()[name].pos[2]))
        if len(hist) > 10 and hist[-11] - hist[-1] < 0.0003 and \
                (hist[-1] - T.TOP) * 1000 - T.COLLAR_Z[1] > T.SEAT_MM:
            stuck = round((hist[-1] - T.TOP) * 1000 - T.COLLAR_Z[1], 2)
            log.setdefault("stalls_mm", []).append(stuck)
            if tries >= RECENTRE:
                arm._solve(arm.pos + [0, 0, 0.002], arm.rot)        # still stuck: back off 2 mm and leave it
                break
            tries += 1
            yield from arm.move(arm.pos + [0, 0, 0.002], arm.rot, 0.3)
            yield from corrected(T.COLLAR_Z[1] + stuck + 2, 0.3, 0.5)
            hist = []
    yield from wait(0.5)
    if variant == "hold":
        yield from wait(4.0)
        return
    yield from arm.set_grip(1.0, 0.6)
    yield from arm.move(arm.pos + [0, 0, 0.06], arm.rot, 1.2)
    yield from arm.move(arm.home[0], arm.home[1], 2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def episode(emb, variant: str, out: Path, max_steps: int, record: bool, hold_s: float = 2.0) -> dict:
    from inspect_robots import Scene

    obs = emb.reset(Scene(id="demo", instruction="demo"))
    T = emb.task.module
    ctx = Ctx(emb)
    log: dict = {}
    gen = script(T, ctx, variant, log)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    debug = os.environ.get("DEMO_DEBUG")
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 100 == 0 or (debug and steps % 10 == 0):
            print(f"step {steps} score {info.get('score')} report {json.dumps(T._STATE['report'])}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "script_end": ended, "variant": variant,
              "state": T.K, "report": T._STATE["report"], "plan": log,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    out.mkdir(parents=True, exist_ok=True)
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), None)
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    print("RESULT " + json.dumps(result, default=float), flush=True)
    return result


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, variants=None) -> list:
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176)
    if not variants:
        return [episode(emb, os.environ.get("DEMO_VARIANT", "") or "correct", Path(out_dir), max_steps, record)]
    return [episode(emb, v, Path(out_dir) / v, max_steps, record) for v in variants]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)  # the run limit
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--variants", default="")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record, variants=[v for v in a.variants.split(",") if v])
    os._exit(0)
