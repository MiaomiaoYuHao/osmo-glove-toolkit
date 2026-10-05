import json, csv, collections
d = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_231202_pose_monitor_trace'

# analysis.csv 行数
n=0; kinds=collections.Counter(); tmin=None; tmax=None
with open(d+r'\analysis.csv', encoding='utf-8', errors='replace') as fh:
    r=csv.DictReader(fh)
    for row in r:
        n+=1
        kinds[row.get('record_kind') or '?'] += 1
        t = row.get('mag_device_time_s') or row.get('quat_device_time_s')
        if t:
            try:
                v=float(t)
                tmin = v if tmin is None else min(tmin,v)
                tmax = v if tmax is None else max(tmax,v)
            except ValueError: pass
print("=== analysis.csv ===")
print("  行数 %d   设备时间 %.1f .. %.1f s  (时长 %.1f s)" % (n, tmin, tmax, tmax-tmin))
dur = tmax-tmin
for k,v in kinds.most_common(6):
    print("    %-8s %6d   (%.1f Hz)" % (k, v, v/dur))

# frames.jsonl
nf=0; fk=collections.Counter()
with open(d+r'\frames.jsonl', encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        nf+=1
        try: o=json.loads(ln)
        except Exception: continue
        fk[o.get('kind') or '?'] += 1
print()
print("=== frames.jsonl ===")
print("  行数 %d" % nf)
for k,v in fk.most_common(8):
    print("    %-14s %6d   (%.1f Hz)" % (k, v, v/dur))

# firmware_diagnostics.jsonl 里三类行
fd=collections.Counter()
with open(d+r'\firmware_diagnostics.jsonl', encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        if '"line":"PARM' in ln or '"line": "PARM' in ln: fd['PARM']+=1
        elif 'YAW2 t=' in ln: fd['YAW2']+=1
        elif 'YAW t=' in ln: fd['YAW']+=1
print()
print("=== firmware_diagnostics.jsonl ===")
for k,v in fd.most_common():
    print("    %-6s %6d   (%.1f Hz)" % (k, v, v/dur))
