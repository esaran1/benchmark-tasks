"""Scripted demo for the three-phase motor task, run in the yam_arena embodiment (closed loop on the measured plug poses).

    python demo.py task.py OUT_DIR [--max-steps 1800] [--no-record]

The correct strategy swaps the leads in sockets U and W (any two would do). The left arm pulls the plug out of U and
holds it aside; the right arm pulls the plug out of W, carries it over the middle lead and pushes it home in U, lets go
and moves away; the left arm then carries its plug over to W and pushes it home, lets go and moves away. Each push is
closed loop on the plug: it is lined up with the socket from its measured position, pushed in, and topped up if it has
slipped in the fingers. Then the robot waits clear of everything while the tester switches the supply on.

Environment: DEMO_VARIANT selects a failure variant (README, Scripted demo):
  cyclic      move all three leads one socket along (U->V, V->W, W->U): the phase sequence is unchanged
  swap_twice  swap U and W, then swap V and W: two swaps, so the order is a cyclic shift of the start
  half_home   the correct swap, but the last plug pushed in only to 11 mm (in its socket, not home)
  unplug_one  the correct swap, but the last plug left lying on the table in front of its socket
  hold        the correct swap, but the left arm keeps hold of the last plug
  none        nothing moved
MOTOR_STATE=k (0-9) runs the gap test's other starting states. DEMO_DEBUG=1 prints the score report every 10 steps.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

HZ = 20
GRASP_BACK = 0.010        # pinch the plug this far from its lead end (m)
SEAT_TO = 0.0245          # push a plug until its tip is this deep (the socket is 26 mm deep)
FINGER_PINCH = -0.020     # finger opening before a pinch (pads about 44 mm apart)
CARRY = 0.040             # carry a plug this much higher than when it rests on the table (m)


class Arm:
    """One arm: smooth straight-down Cartesian moves (IK on the URDF) from wherever its last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena.constants import FINGER_OPEN, READY

        self.name = name
        self.base = np.asarray(base_pos, dtype=float)
        self.q = np.array(READY, dtype=float)
        self.finger = FINGER_OPEN
        self.cmd = np.r_[self.q, 1.0]
        self.yaw = 0.0

    def tcp(self) -> np.ndarray:
        from yam_arena import ik

        return ik.tcp_pose(self.q, self.base)[0]

    def move(self, pos, *, yaw: float | None = None, finger: float | None = None, duration: float = 1.0):
        """Generator: go to ``pos`` (world m, TCP) with a straight-down grip turned ``yaw``; ``finger`` in m."""
        from yam_arena.demo import ArmPlan, Waypoint

        f = self.finger if finger is None else finger
        y = self.yaw if yaw is None else yaw
        plan = ArmPlan([Waypoint(duration, tuple(map(float, pos)), f, "", y)], self.base, q_start=self.q,
                       finger_start=self.finger)
        for k in range(1, max(1, round(duration * HZ)) + 1):
            self.cmd, _ = plan.action7(k / HZ)
            self.q = plan.q.copy()
            yield
        self.finger, self.yaw = f, y

    def grip(self, finger: float, duration: float = 0.6):
        yield from self.move(self.tcp(), finger=finger, duration=duration)


def wait(s: float):
    for _ in range(max(1, round(s * HZ))):
        yield


def both(a, b):
    """Run two generators in lockstep until both end."""
    live = [iter(a), iter(b)]
    while live:
        for g in list(live):
            try:
                next(g)
            except StopIteration:
                live.remove(g)
        if live:
            yield


class Ctx:
    def __init__(self, T, emb):
        self.T, self.emb = T, emb

    def states(self):
        return self.T.objects_now(self.emb._env)

    def plug(self, lead: str):
        return self.T.plug_pose(self.states()[f"lead_{lead}"])

    def lead_in(self, socket: int) -> str:
        """The lead whose plug is in ``socket`` now."""
        st = self.states()
        for n in self.T.LEADS:
            if self.T.socket_of(st[f"lead_{n}"])[0] == socket:
                return n
        raise RuntimeError(f"no plug in socket {socket}")

    def depth(self, lead: str, socket: int) -> float:
        j, d = self.T.socket_of(self.states()[f"lead_{lead}"])
        return d / 1000 if j == socket else -1.0


def plug_local(ctx) -> dict:
    """Debug: each plug in the drive frame: tip x (mm, + inside the face), centre y, centre z above the table (mm),
    heading (deg)."""
    T, st, out = ctx.T, ctx.states(), {}
    for n in T.LEADS:
        p, a = T.plug_pose(st[f"lead_{n}"])
        pl, al = T.to_drive(p), T.drive_rot().T @ a
        tip = pl + al * T.PL / 2
        out[n] = [round(float(tip[0]) * 1000, 1), round(float(pl[1]) * 1000, 1), round(float(pl[2]) * 1000, 1),
                  round(math.degrees(math.atan2(al[1], al[0])), 1), round(math.degrees(math.asin(np.clip(al[2], -1, 1))), 1)]
    return out


def _gz(T) -> float:
    return T.TOP + T.PH + 0.003


def pick(ctx, arm, lead: str):
    """Pinch the lead's plug GRASP_BACK from its lead end, pull it straight out of its socket, lift it."""
    from yam_arena.constants import FINGER_CLOSED

    T = ctx.T
    p, a = ctx.plug(lead)
    yaw = T.YAW
    g = p - a * (T.PL / 2 - GRASP_BACK)
    g = np.array([g[0], g[1], _gz(T)])
    yield from arm.move(g + [0, 0, 0.06], yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move(g, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    out = g - a * 0.05
    out[2] = g[2]
    yield from arm.move(out, duration=1.5)
    yield from arm.move(out + [0, 0, CARRY], duration=0.8)


def carry_to(ctx, arm, socket: int, pre: float = 0.008, duration: float = 2.0):
    """Carry the held plug to CARRY above the spot in front of ``socket`` (tip ``pre`` short of the face)."""
    T = ctx.T
    ax = T.drive_rot()[:, 0]
    g = T.socket_mouth(socket) + ax * (-pre - T.PL / 2) - ax * (T.PL / 2 - GRASP_BACK)
    yield from arm.move((g[0], g[1], _gz(T) + CARRY), duration=duration)


def insert(ctx, arm, lead: str, socket: int, seat_to: float = SEAT_TO, pre: float = 0.008):
    """Lower the held plug in front of ``socket``, line it up from its measured position, push it in until its tip is
    ``seat_to`` deep (topping up if it slipped in the fingers). Returns the final grasp point."""
    T = ctx.T
    R = T.drive_rot()
    ax, ay = R[:, 0], R[:, 1]
    mouth = T.socket_mouth(socket)
    tcp = arm.tcp()
    p, _a = ctx.plug(lead)
    off = (tcp - p)[:2] @ ax[:2]                     # grasp point to plug centre along the socket axis, as held
    g0 = mouth + ax * (-pre - T.PL / 2 + off)
    g0[2] = _gz(T)
    yield from arm.move(g0 + [0, 0, CARRY], duration=0.6)
    yield from arm.move(g0, duration=0.8)
    yield from wait(0.3)
    p, _a = ctx.plug(lead)
    high = float(p[2] - (T.TOP + T.PH / 2 + 0.0005))   # held higher than resting on the table: lower it (<= 3 mm)
    if high > 0.001:
        g0 = g0 - [0, 0, min(high, 0.003)]
        yield from arm.move(g0, duration=0.4)
    dl = 0.0

    def align(n):
        """Line the plug's tip up with the socket's centre line (the arm sags and the lead drags it sideways)."""
        nonlocal dl
        for _ in range(n):
            p, a = ctx.plug(lead)
            tip = p + a * T.PL / 2
            err = float((mouth - tip)[:2] @ ay[:2])
            if abs(err) < 0.0007:
                break
            dl = float(np.clip(dl + err, -0.02, 0.02))
            yield from arm.move(g0 + ay * dl, duration=0.5)
            yield from wait(0.2)

    yield from align(3)
    g0 = g0 + ax * (pre - 0.002)                     # slide on until the tip is 2 mm short of the face; line up again
    pre = 0.002
    yield from arm.move(g0 + ay * dl, duration=0.6)
    yield from wait(0.2)
    yield from align(3)
    for _attempt in range(3):                        # enter 5 mm; if the tip did not go in, back off and line up again
        yield from arm.move(g0 + ay * dl + ax * (pre + 0.005), duration=0.8)
        yield from wait(0.2)
        if ctx.depth(lead, socket) >= 0.002:
            break
        yield from arm.move(g0 + ay * dl, duration=0.6)
        yield from align(3)
    g = g0 + ay * dl + ax * (seat_to + pre)
    yield from arm.move(g, duration=1.5)
    for _ in range(2):                               # top up if the plug slipped back in the fingers
        d = ctx.depth(lead, socket)
        if d >= seat_to - 0.002:
            break
        g = g + ax * (seat_to - max(d, 0.0))
        yield from arm.move(g, duration=1.0)
    yield from wait(0.3)
    return g


def let_go(ctx, arm, park):
    """Open, lift straight up, and move to ``park`` (clear of the drive, the gland and the leads)."""
    p = arm.tcp()
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move(p + [0, 0, 0.08], duration=1.0)
    yield from arm.move(park, duration=1.5)


def move_plug(ctx, arm, lead, socket, park, seat_to=SEAT_TO, carry_s=2.0):
    """Pick the lead's plug, carry it to ``socket``, push it in, let go and park."""
    yield from pick(ctx, arm, lead)
    yield from carry_to(ctx, arm, socket, duration=carry_s)
    yield from insert(ctx, arm, lead, socket, seat_to)
    yield from let_go(ctx, arm, park)


def strategy(ctx, arms, name):
    """Generator: yield once per control step. name = "correct" or a failure variant."""
    T = ctx.T
    L, Rt = arms["left"], arms["right"]
    top = T.TOP
    park_l, park_r = np.array([0.20, 0.22, top + 0.16]), np.array([0.20, -0.22, top + 0.16])
    hold_l = np.array([0.24, 0.17, _gz(T) + CARRY])       # where the left arm holds a plug aside
    if name == "none":
        yield from wait(3.0)
        return
    u, v, w = ctx.lead_in(0), ctx.lead_in(1), ctx.lead_in(2)
    if name == "cyclic":                                   # U->V, V->W, W->U
        yield from pick(ctx, Rt, w)                        # right holds W's plug aside
        yield from Rt.move((0.24, -0.17, _gz(T) + CARRY), duration=1.5)
        yield from move_plug(ctx, L, v, 2, np.array([0.24, 0.10, top + 0.10]))   # V's plug into W
        yield from move_plug(ctx, L, u, 1, park_l)         # U's plug into V
        yield from carry_to(ctx, Rt, 0)
        yield from insert(ctx, Rt, w, 0)                   # W's plug into U
        yield from let_go(ctx, Rt, park_r)
        yield from wait(1.0)
        return
    # the correct swap (and the variants built on it): U <-> W
    yield from pick(ctx, L, u)
    yield from L.move(hold_l, duration=1.5)
    yield from move_plug(ctx, Rt, w, 0, park_r)            # W's plug into U
    last_seat = 0.011 if name == "half_home" else SEAT_TO
    yield from carry_to(ctx, L, 2, duration=2.5)
    if name == "unplug_one":                               # lay it on the table, tip 10 mm short of W
        R = T.drive_rot()
        ax = R[:, 0]
        p, _a = ctx.plug(u)
        off = (L.tcp() - p)[:2] @ ax[:2]
        g = T.socket_mouth(2) + ax * (-0.010 - T.PL / 2 + off)
        g[2] = _gz(T)
        yield from L.move(g + [0, 0, CARRY], duration=0.6)
        yield from L.move(g, duration=0.8)
        yield from let_go(ctx, L, park_l)
        yield from wait(1.0)
        return
    yield from insert(ctx, L, u, 2, last_seat)             # U's plug into W
    if name == "hold":
        yield from wait(6.0)
        return
    yield from let_go(ctx, L, park_l)
    if name == "swap_twice":                               # then swap V and W as well
        yield from pick(ctx, L, v)
        yield from L.move(hold_l, duration=1.5)
        yield from move_plug(ctx, Rt, u, 1, park_r)        # W's plug (now u) into V
        yield from carry_to(ctx, L, 2, duration=2.5)
        yield from insert(ctx, L, v, 2)
        yield from let_go(ctx, L, park_l)
    yield from wait(1.0)


def _write_mp4(frames, path: Path) -> None:
    h, w = frames[0].shape[:2]
    proc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(HZ),
         "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
        stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(np.asarray(f)[..., :3], dtype=np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, hold_s: float = 8.0) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    w, h = (320, 176) if record else (64, 36)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=HZ, cam_width=w, cam_height=h)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    T = emb.task.module
    ctx = Ctx(T, emb)
    arms = {s: Arm(s, station.BASES[s]) for s in ("left", "right")}
    variant = os.environ.get("DEMO_VARIANT", "") or "correct"
    gen = strategy(ctx, arms, variant)
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
            print(f"step {steps} score {info.get('score')} report {json.dumps(T._STATE['report'], default=str)}",
                  flush=True)
            if debug:
                print(f"step {steps} local {json.dumps(plug_local(ctx))} tcp L {np.round(arms['left'].tcp() * 1000).tolist()}"
                      f" R {np.round(arms['right'].tcp() * 1000).tolist()}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "script_end_step": ended, "variant": variant,
              "state": T.K, "report": T._STATE["report"]}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
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
