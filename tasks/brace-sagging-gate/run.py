"""Gate-brace stage-1 physics runs: python run.py OUT_DIR  (BRACE_MODE=none|comp|tens, other BRACE_* env)."""
import json, os, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from run_common import HZ, Robot, make, states  # noqa: E402


def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    n = int(os.environ.get("BRACE_STEPS", "200"))
    t0 = time.time()
    emb, T = make("task.py")
    from partkit import body_poses
    rob = Robot()
    trace = []
    for k in range(n + 1):
        if k:
            emb.step(rob.action())
        if k % 5 == 0:
            bp = body_poses(emb._env, "gate")
            row = {"step": k, "sag_mm": round(T.sag_mm(bp), 2)}
            if T.MODE != "none":
                b = states(emb)["brace"]
                row["brace"] = [round(v * 1000, 1) for v in (np.asarray(b.pos) - [T.FX, 0, T.TOP])]
                row["brace_rx_deg"] = round(float(np.degrees(2 * np.arctan2(b.quat_xyzw[0], b.quat_xyzw[3]))), 2)
            trace.append(row)
    rec = {"mode": T.MODE, "L_brace_mm": round(T.L_BRACE * 1000, 2), "final": trace[-1], "first": trace[:6],
           "wall_s": round(time.time() - t0, 1)}
    (out / "trace.json").write_text(json.dumps(trace)); (out / "result.json").write_text(json.dumps(rec))
    print("RESULT " + json.dumps(rec), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
