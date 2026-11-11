"""Drop test (no robot motion): for each case, write the slip yoke 5 mm above the splines' top, centred on the shaft,
with its ears at a given angle from the far yoke's ear line, and let it fall for 3 s; print where it ends up and its
placement score (the release and stillness checks aside).

    DRIVE_STATE=k python stage1.py OUT.json

Cases (degrees from the far yoke's ear line): the six spline positions 0, 60, ..., 300, and 3, -4, 15, 30, 90, which
are between spline positions.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from run_common import Robot, make, states  # noqa: E402

CASES = [0, 60, 120, 180, 240, 300, 3, -4, 30, 90, 15]


def main():
    out = Path(sys.argv[1])
    t0 = time.time()
    emb, T = make("task.py")
    from partkit import write_pose
    rob = Robot()
    res = []
    for rel in CASES:
        yaw = T.PHI1 + math.radians(rel) - math.pi / 2          # part yaw so its ears (+y) point at PHI1 + rel
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        write_pose(emb._env, T.SLEEVE.name, (T.FX, T.FY, T.TOP + (T.SPLINE_Z[1] + 5) / 1000), q)
        traj = []
        for k in range(60):                                      # 3 s
            emb.step(rob.action())
            if k % 10 == 9:
                traj.append(T.sleeve_eval(states(emb)[T.SLEEVE.name])["height_mm"])
        e = T.sleeve_eval(states(emb)[T.SLEEVE.name])
        e.update(score=T.placement_score(e), rel=rel, traj_h=traj)
        print("CASE", json.dumps(e), flush=True)
        res.append(e)
    out.write_text(json.dumps({"state": T.K, "cases": res, "wall_s": round(time.time() - t0, 1)}))
    os._exit(0)


if __name__ == "__main__":
    main()
