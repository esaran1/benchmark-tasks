"""compile.py OUT_DIR EXPECTED.json  ->  table of task_score per state x strategy, and mismatches.
EXPECTED.json: {"strategy": expected_score, ...} (written BEFORE the batch, never loosened without data).
Pass rule printed at the end: every run present and |score - expected| <= 1e-9."""
import json, os, sys
O, E = sys.argv[1], json.load(open(sys.argv[2]))
S = list(E)
ks = sorted({int(n.split("_")[0][1:]) for n in os.listdir(O) if n.startswith("s") and "_" in n})
print("state " + " ".join(f"{s:>12}" for s in S))
bad, missing = [], 0
for k in ks:
    row = []
    for s in S:
        p = f"{O}/s{k}_{s}/result.json"
        if not os.path.exists(p):
            row.append(f"{'-':>12}"); missing += 1; continue
        r = json.load(open(p)); sc = float(r["task_score"]); row.append(f"{sc:12.3f}")
        if abs(sc - E[s]) > 1e-9:
            bad.append((k, s, sc, r.get("report")))
    print(f"s{k:<4} " + " ".join(row))
print("mismatches:", json.dumps(bad, default=str))
print("PASS" if not bad and not missing and len(ks) >= 10 else f"NOT PASS (missing {missing}, states {len(ks)})")
