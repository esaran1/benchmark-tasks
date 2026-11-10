"""Gate-brace stage-2 robot runs.  python run2.py STRATEGY OUT_DIR   (BRACE_STATE=k)

Strategies:
  correct      latch arm lifts the latch side 4 mm past square, brace arm fits the brace on the compression diagonal
               (bottom of the hinge stile to top of the latch stile), latch arm lowers the gate onto it; both let go
  comp_nolift  the brace pushed at the compression diagonal without lifting the gate (it is too long to fit)
  tens_nolift  the brace fitted on the other diagonal (top hinge to bottom latch), no lift
  tens_lift    lift to square, brace on the other diagonal, lower, let go
  lift_only    lift to square, hold 3 s, let go (no brace)
  hold         the correct fit, but the latch arm keeps hold of the handle to the end
  none         nothing moved
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from run_common import HZ, make, states  # noqa: E402


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


def main():
    name, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    emb, T = make("task.py")
    t_ready = time.time()
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from partkit import body_poses
    from yam_arena import station

    arms = {s: Arm(s, station.BASES[s]) for s in ("left", "right")}
    gen = strategy(T, emb, arms, name)
    steps, ended, trace = 0, None, []
    hold_end = int(float(os.environ.get("BRACE_HOLD_S", "5")) * HZ)
    while True:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        emb.step(Action(data=pack(arms["left"].cmd, arms["right"].cmd)))
        steps += 1
        if steps % 10 == 0:
            m = T.measure(states(emb), body_poses(emb._env, "gate"))
            m["step"] = steps
            trace.append(m)
            if os.environ.get("BRACE_DEBUG"):
                b = states(emb)["brace"]
                ba = arms["left"] if T.SGN > 0 else arms["right"]
                m["brace_pos"] = [round(float(v) * 1000, 1) for v in b.pos]
                m["tcp_cmd"] = [round(float(v) * 1000, 1) for v in ba.pos]
                m["grip_cmd"] = round(ba.grip, 2)
                from yam_arena import ik
                art = emb._env.unwrapped.scene[ba.name + "_robot"]
                qa = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
                m["tcp_act"] = [round(float(v) * 1000, 1) for v in ik.tcp_pose(qa[:6], ba.base)[0]]
                m["q_act"] = [round(float(v), 3) for v in qa]
                if 150 < steps < 260:
                    shp = T.robot_shapes()
                    near = []
                    for nm, pts_l, r, pp, *_ in shp:
                        w = pts_l @ r.T + pp
                        for on, o in states(emb).items():
                            d = float(np.linalg.norm(w - np.asarray(o.pos), axis=1).min() * 1000)
                            near.append((round(d, 1), nm, on))
                    m["near"] = sorted(near)[:4]
                print("TR", json.dumps(m), flush=True)
        if ended is not None and steps - ended >= hold_end:
            break
        if steps > 3000:
            break
    st, bp = states(emb), body_poses(emb._env, "gate")
    m = T.measure(st, bp)
    gap = T.robot_gap_mm(T.robot_shapes(), st, bp)
    last = [r["sag_mm"] for r in trace[-5:]]
    rec = {"strategy": name, "state": T.K, "final": m, "robot_gap_mm": round(gap, 1) if math.isfinite(gap) else None,
           "sag_range_last1s": [min(last), max(last)], "steps": steps,
           "worst_ik_mm": {s: round(a.worst_mm, 1) for s, a in arms.items()},
           "wall_s": round(time.time() - t0, 1), "sim_steps_per_s": round(steps / (time.time() - t_ready), 2)}
    (out / "result.json").write_text(json.dumps(rec))
    (out / "trace.json").write_text(json.dumps(trace))
    print("RESULT " + json.dumps(rec), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
