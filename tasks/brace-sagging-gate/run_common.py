"""Sling gate-test runner.  python run.py hang OUT.json   (SLING_PAIR, SLING_COM, ... env)"""
from __future__ import annotations
import json, os, sys, time
from pathlib import Path
import numpy as np
HZ = 20
HERE = Path(__file__).resolve().parent


def make(task_path="task.py"):
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    sys.path.insert(0, str(HERE))
    emb = yam_arena_embodiment(task=str(HERE / task_path), object_jitter=0, control_hz=HZ, cam_width=320, cam_height=176)
    emb.reset(Scene(id="s", instruction="s"))
    return emb, emb.task.module


class Robot:
    def __init__(self):
        from yam_arena.constants import READY
        self.cmd = np.r_[np.array(READY, float), 1.0]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.cmd, self.cmd))


def states(emb):
    from yam_arena.scene import object_states
    return object_states(emb._env, emb.task.names)


def main():
    mode, out = sys.argv[1], Path(sys.argv[2])
    t0 = time.time()
    emb, T = make(os.environ.get("SLING_TASK", "task.py"))
    rob = Robot()
    from partkit import write_pose
    steps, trace = 0, []

    def step():
        nonlocal steps
        emb.step(rob.action()); steps += 1
        if steps % 5 == 0:
            st = states(emb)
            b = st["bar"]
            trace.append((steps, round(T.bar_tilt_deg(b), 2), round((b.pos[2] - T.TOP) * 1000, 1)))
    if mode == "still":
        from isaaclab.sim.utils.stage import get_current_stage
        stage = get_current_stage()
        print("CK", T.cablekit.__file__, flush=True)
        for prim in stage.Traverse():
            pth = str(prim.GetPath())
            if pth.endswith("/anchor") or pth.endswith("leg_0") or pth.endswith("leg_1"):
                a = {n: str(prim.GetAttribute(n).Get()) for n in ("physics:localPos0", "physics:localRot0", "xformOp:translate", "xformOp:orient") if prim.GetAttribute(n)}
                r = prim.GetRelationship("physics:body0")
                print("PRIM", pth, a, r.GetTargets() if r else None, flush=True)
        sc = emb._env.unwrapped.scene
        for nm in ("leg_0", "leg_1"):
            a = sc[nm]
            print("ART", nm, id(a), a.cfg.prim_path, a.root_physx_view.prim_paths[:1] if hasattr(a, "root_physx_view") else "", getattr(a.data.root_pos_w, "torch", a.data.root_pos_w)[0].tolist(), flush=True)
        st = states(emb)
        print("T0 " + json.dumps({l.name: [[round(v * 1000, 1) for v in (p - [T.FX, T.FY, T.TOP])] for p in st[l.name].points] for l in T.LEGS}), flush=True)
        for _ in range(3 * HZ):
            step()
        st = states(emb)
        r = {k: [round(v * 1000, 1) for v in (np.asarray(st[k].pos) - [T.FX, T.FY, T.TOP])] for k in ("bar", "hookbar")}
        for l in T.LEGS:
            r[l.name + "_toggle"] = [round(v * 1000, 1) for v in (np.asarray(st[l.name].end_plug) - [T.FX, T.FY, T.TOP])]
            r[l.name + "_pts"] = [[round(v * 1000, 1) for v in (p - [T.FX, T.FY, T.TOP])] for p in st[l.name].points]
        r["tilt"] = round(T.bar_tilt_deg(st["bar"]), 2); r["wall_s"] = round(time.time() - t0, 1)
        print("RESULT " + json.dumps(r), flush=True)
    if mode == "hang":
        for _ in range(2 * HZ):
            step()
        for d in T.DUNNAGE:
            write_pose(emb._env, d.name, (d.pos[0], d.pos[1], -0.5), (0, 0, 0, 1))
        for _ in range(int(float(os.environ.get("SLING_HANG_S", "8")) * HZ)):
            step()
        st = states(emb)
        b = st["bar"]
        tog = [np.asarray(st[l.name].end_plug) for l in T.LEGS]
        tilts = [r[1] for r in trace if r[0] > steps - 2 * HZ]
        rec = {"mode": mode, "pair": T.PAIR, "lugs_mm": [T.LUGS[i] for i in T.PAIR], "com": T.COM,
               "tilt_final": round(T.bar_tilt_deg(b), 2), "tilt_last2s_range": [min(tilts), max(tilts)],
               "bar_z_mm": round((b.pos[2] - T.TOP) * 1000, 1), "bar_y_mm": round((b.pos[1] - T.FY) * 1000, 1),
               "hook_mm": [round(v * 1000, 1) for v in T.HOOK_PT], "toggles_y_mm": [round((t[1] - T.FY) * 1000, 1) for t in tog], "toggles_z_mm": [round((t[2] - T.TOP) * 1000, 1) for t in tog], "steps": steps, "wall_s": round(time.time() - t0, 1)}
        out.write_text(json.dumps({"rec": rec, "trace": trace}))
        print("RESULT " + json.dumps(rec), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
