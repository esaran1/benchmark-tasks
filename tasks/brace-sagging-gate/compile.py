import json, os
B = os.path.expanduser("~/brace_work/gate")
S = ["correct", "comp_nolift", "tens_nolift", "tens_lift", "lift_only", "hold", "none"]
EXP = {"correct": 1.0}
def score(r):
    f = r["final"]; gap = r["robot_gap_mm"]; lo, hi = r["sag_range_last1s"]
    if not f.get("seated") or (gap is not None and gap < 20) or hi - lo > 0.2:
        return 0.0
    d = f["sag_mm"]
    return 1.0 if d <= 3 else 0.5 if d <= 6 else 0.2 if d <= 9 else 0.0
print("state " + " ".join(f"{s:>11}" for s in S))
bad, sags = [], []
for k in range(10):
    row = []
    for s in S:
        p = f"{B}/s{k}_{s}/result.json"
        if not os.path.exists(p): row.append(f"{'-':>11}"); continue
        r = json.load(open(p)); sc = score(r); row.append(f"{sc:4.1f}/{r['final']['sag_mm']:5.1f}")
        if s == "correct": sags.append(r["final"]["sag_mm"])
        if abs(sc - EXP.get(s, 0.0)) > 1e-9: bad.append((k, s, sc, r["final"], r["robot_gap_mm"]))
    print(f"s{k:<4} " + " ".join(f"{c:>11}" for c in row))
print("cells: score/sag_mm.  mismatches:", bad)
print("correct sags:", sags)
