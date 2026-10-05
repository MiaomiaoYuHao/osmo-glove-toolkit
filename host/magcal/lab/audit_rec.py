import json, numpy as np
d = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_225025_pose_monitor_trace'
# analysis.csv 行数 + 各类记录数
import csv
n=0; kinds={}
with open(d+r'\analysis.csv', encoding='utf-8', errors='replace') as fh:
    r=csv.DictReader(fh)
    for row in r:
        n+=1
        k=row.get('record_kind') or row.get('kind') or '?'
        kinds[k]=kinds.get(k,0)+1
print("analysis.csv 行数:", n)
print("  record_kind 分布:", dict(sorted(kinds.items(), key=lambda x:-x[1])[:8]))
# frames.jsonl
nf=0; fk={}
with open(d+r'\frames.jsonl', encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        nf+=1
        try: o=json.loads(ln)
        except Exception: continue
        k=o.get('kind') or o.get('record_kind') or '?'
        fk[k]=fk.get(k,0)+1
print("frames.jsonl 行数:", nf)
print("  kind 分布:", dict(sorted(fk.items(), key=lambda x:-x[1])[:8]))
# stages.jsonl
ns=0; sk={}
with open(d+r'\stages.jsonl', encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        ns+=1
        try: o=json.loads(ln)
        except Exception: continue
        k=o.get('stage') or '?'
        sk[k]=sk.get(k,0)+1
print("stages.jsonl 行数:", ns)
print("  stage 分布:", dict(sorted(sk.items(), key=lambda x:-x[1])[:8]))
