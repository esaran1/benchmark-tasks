"""compile.py OUT_DIR EXPECTED.json  ->  table of task_score per state x strategy (with where each lead ended), and
mismatches. EXPECTED.json: {"strategy": expected_score, ...}, written before the batch (gap_rules.txt).
Pass rule: every run present, |score - expected| <= 1e-9, on at least 10 states."""
import json
import os
import sys

O, E = sys.argv[1], json.load(open(sys.argv[2]))
S = list(E)
ks = sorted({int(n.split("_")[0][1:]) for n in os.listdir(O) if n.startswith("s") and "_" in n})
print("score (sockets of brown/black/grey, tip depths mm)")
print("state " + " ".join(f"{s:>12}" for s in S))
bad, missing, detail = [], 0, []
for k in ks:
    row = []
    for s in S:
        p = f"{O}/s{k}_{s}/result.json"
        if not os.path.exists(p):
            row.append(f"{'-':>12}")
            missing += 1
            continue
        r = json.load(open(p))
        sc = float(r["task_score"])
        row.append(f"{sc:12.3f}")
        leads = r.get("report", {}).get("leads", {})
        detail.append((k, s, sc, r.get("steps"), {n: v for n, v in leads.items()}, r.get("report", {}).get("tote_mm")))
        if abs(sc - E[s]) > 1e-9:
            bad.append((k, s, sc, r.get("report")))
    print(f"s{k:<4} " + " ".join(row))
print("\nper run: state strategy score steps leads(socket, tip mm) tote_mm")
for d in detail:
    print(*d)
print("mismatches:", json.dumps(bad, default=str))
print("PASS" if not bad and not missing and len(ks) >= 10 else f"NOT PASS (missing {missing}, states {len(ks)})")
