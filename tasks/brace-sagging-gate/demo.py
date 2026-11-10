"""Scripted demo for the gate-brace task, run in the yam_arena embodiment (closed loop on the measured sag and brace
pose).

    python demo.py task.py OUT_DIR [--max-steps 1800] [--no-record]

The latch-side arm grips the handle on the latch-side stile and lifts that side, closed loop on the measured sag,
until it is 4 mm above square. The other arm picks the brace up from the table at its middle, turns it to lie along the
compression diagonal (the bottom hinge-side seat to the top latch-side seat), brings it in front of the seats,
corrects its aim twice from the brace's measured pose and pushes it into the seats. The latch arm lowers the gate onto
the brace (to 3 mm below square, which the brace stops), the brace arm lets go and backs off, then the latch arm lets go
of the handle; both go back to where they started.

Environment: DEMO_VARIANT selects a failure variant (README, Scripted demo):
  comp_nolift  the brace pushed at the compression diagonal without lifting the gate (it is too long to go in)
  tens_nolift  the brace fitted on the other diagonal (top hinge-side to bottom latch-side corner), no lift
  tens_lift    lift to square, the brace on the other diagonal, lower, let go of both
  lift_only    lift to square, hold 3 s, let go (no brace)
  hold         the correct fit, but the latch arm keeps hold of the handle to the end
  none         nothing moved
BRACE_STATE=k (0-9) runs the gap test's other starting states. DEMO_DEBUG=1 prints the score report every 10 steps.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HZ = 20


def states(emb):
    from yam_arena.scene import object_states

    return object_states(emb._env, emb.task.names)


class Arm:
    def __init__(self, name, base_pos):
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
        if pe > 0.001:
            q2, pe2, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(READY, float), base_pos=self.base, iters=200)
            if pe2 < pe:
                q, pe = q2, pe2
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration, grip=None):
        from yam_arena import ik
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def grip_to(self, g, duration=0.8):
        yield from self.move(self.pos, self.rot, duration, g)

    def aim(self, pos, rot=None):
        self._solve(np.asarray(pos, float), self.rot if rot is None else rot)


def wait(s):
    for _ in range(max(1, round(s * HZ))):
        yield


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def strategy(T, emb, arms, name):
    from partkit import body_poses
    from yam_arena.demo import grip_rotation

    latch_arm = arms["right"] if T.SGN > 0 else arms["left"]     # the latch stile is on the -SGN side
    brace_arm = arms["left"] if T.SGN > 0 else arms["right"]
    yaw_b = -math.pi / 2 if brace_arm.name == "left" else math.pi / 2
    yaw_h = math.pi / 2 if latch_arm.name == "right" else -math.pi / 2
    down_b, down_h = grip_rotation(yaw_b), grip_rotation(yaw_h)
    if name == "none":
        yield from wait(3.0)
        return

    def bodies():
        return body_poses(emb._env, "gate")

    def sag():
        return T.sag_mm(bodies())

    handle = np.array([T.FX, T.HANDLE_Y - T.SGN * 0.01, T.TOP + T.HANDLE_Z])   # grip 10 mm out from the handle centre

    def lift_to(target_mm, seconds):
        """Closed loop on the measured sag: move the handle until the latch side is target_mm below square."""
        for _ in range(round(seconds * HZ)):
            err = sag() - target_mm                     # mm; positive: too low
            latch_arm.aim(latch_arm.pos + [0, 0, float(np.clip(err, -2, 2)) * 0.0004])
            yield

    def take_handle():
        yield from latch_arm.move(handle + [0, 0, 0.05], down_h, 2.0, grip=1.0)
        yield from latch_arm.move(handle, down_h, 1.0)
        yield from latch_arm.grip_to(0.0, 0.8)

    def fit(diag, push=True):
        """Brace arm: pick the brace, turn it to lie along ``diag``'s seats, push it in front-on."""
        b = states(emb)["brace"]
        g = np.asarray(b.pos) + [0, 0, 0.007]
        # first back away from the frame plane to the brace's x, then across: the straight line from home to the brace
        # passes the hinge stile
        yield from brace_arm.move(np.array([g[0], brace_arm.pos[1], g[2] + 0.12]), down_b, 1.2, grip=1.0)
        yield from brace_arm.move(g + [0, 0, 0.06], down_b, 1.2)
        yield from brace_arm.move(g, down_b, 1.0)
        yield from brace_arm.grip_to(0.0, 0.8)
        yield from brace_arm.move(g + [0, 0, 0.09], down_b, 1.2)
        seats = T.seat_points_world(bodies())
        k0 = "bh" if diag == "comp" else "th"
        a, c = seats[k0], seats[T.OPPOSITE[k0]]
        u = (c - a) / np.linalg.norm(c - a)
        theta = math.atan2(u[2], u[1])                  # brace direction in the y-z plane, from +y
        if theta > math.pi / 2:
            theta -= math.pi
        if theta < -math.pi / 2:
            theta += math.pi
        R = Rx(theta) @ Ry(-math.pi / 2) @ down_b
        mid = (a + c) / 2
        front = np.array([T.BRACE_X - 0.07, mid[1], mid[2]])
        yield from brace_arm.move(front, R, 2.5)
        for _ in range(2):                              # correct from the measured brace pose
            yield from wait(0.6)
            bs = states(emb)["brace"]
            off = np.asarray(bs.pos) - brace_arm.pos
            tgt = front - off
            yield from brace_arm.move(np.array([brace_arm.pos[0], tgt[1], tgt[2]]), R, 0.8)
        if push:
            yield from brace_arm.move(brace_arm.pos + [T.BRACE_X - front[0], 0, 0], R, 2.0)
            yield from wait(0.5)

    def let_go(arm, back):
        yield from arm.grip_to(1.0, 0.6)
        yield from arm.move(arm.pos + np.asarray(back), arm.rot, 1.2)
        yield from arm.move(arm.home[0], arm.home[1], 2.0)

    if name in ("correct", "hold", "tens_lift", "lift_only"):
        yield from take_handle()
        target = -4.0 if name in ("correct", "hold") else 0.0
        yield from lift_to(target, 3.0)
    if name == "lift_only":
        yield from wait(3.0)
        yield from let_go(latch_arm, [0, 0, 0.05])
        return
    diag = "comp" if name in ("correct", "hold", "comp_nolift") else "tens"
    yield from fit(diag)
    if name in ("correct", "hold", "tens_lift"):
        yield from lift_to(3.0, 2.5)                    # lower the gate onto the brace
    yield from let_go(brace_arm, [-0.08, 0, 0])
    if name == "hold":
        yield from wait(4.0)
        return
    if name in ("correct", "tens_lift"):
        yield from let_go(latch_arm, [0, 0, 0.05])


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])




def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, hold_s: float = 5.0) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=HZ, cam_width=320, cam_height=176)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    T = emb.task.module
    arms = {s: Arm(s, station.BASES[s]) for s in ("left", "right")}
    variant = os.environ.get("DEMO_VARIANT", "") or "correct"
    gen = strategy(T, emb, arms, variant)
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
        res = emb.step(Action(data=pack(arms["left"].cmd, arms["right"].cmd)))
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 100 == 0 or (debug and steps % 10 == 0):
            print(f"step {steps} score {info.get('score')} report {json.dumps(T._STATE['report'])}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": variant, "state": T.K,
              "report": T._STATE["report"], "worst_ik_mm": {s: round(a.worst_mm, 2) for s, a in arms.items()}}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), None)
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1800)  # the run limit
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
    os._exit(0)
