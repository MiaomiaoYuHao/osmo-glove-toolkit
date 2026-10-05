import json, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\firmware_diagnostics.jsonl'
rows=[]
with open(p, encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        try: o=json.loads(ln)
        except Exception: continue
        pa=o.get('parsed') or {}
        if pa.get('type')!='mag_status_row': continue
        f=pa.get('fields') or {}
        rows.append((o.get('rx_wall_time'), f))
print("P 表快照数:", len(rows))
if not rows: raise SystemExit
t0 = rows[0][0]
ts = np.array([r[0]-t0 for r in rows])
def g(k):
    return np.array([float((r[1].get(k) if r[1].get(k) is not None else np.nan)) for r in rows])
datum = g('datum_deg'); yawc = g('yaw_corr_deg')
w     = g('weight_pct'); st = [r[1].get('status') for r in rows]
geo   = g('geo_x1000'); en = g('e_norm_x1000'); edi = g('e_dip_x1000')
dipv  = [r[1].get('dip_valid') for r in rows]

print("时间跨度 %.1f s，采样约每 %.1f s 一次" % (ts[-1], np.median(np.diff(ts))))
print()
print("=== datum（基准偏置，固件说【绝不能移动】）===")
d0 = datum - datum[0]
print("  首值 %+7.2f°   末值 %+7.2f°   总变化 %+7.2f°" % (datum[0], datum[-1], datum[-1]-datum[0]))
print("  min %+7.2f  max %+7.2f  极差 %.2f°" % (datum.min(), datum.max(), datum.max()-datum.min()))
print("  时序:")
N=min(24,len(rows))
for k in range(N):
    i = k*(len(rows)-1)//(N-1) if N>1 else 0
    print("     t=%6.1f s   datum %+8.2f°  yawc %+8.2f°  status=%-8s w=%3.0f%% geo=%4.0f eN=%4.0f eDi=%4.0f dip%s" %
          (ts[i], datum[i], yawc[i], st[i], w[i], geo[i], en[i], edi[i], "OK" if dipv[i] else "未建"))
print()
print("=== yawc（输出 yaw 的修正量）===")
print("  首值 %+7.2f°   末值 %+7.2f°   总变化 %+7.2f°" % (yawc[0], yawc[-1], yawc[-1]-yawc[0]))
print("  min %+7.2f  max %+7.2f" % (yawc.min(), yawc.max()))
print()
print("=== 状态分布 ===")
from collections import Counter
for k,v in Counter(st).most_common():
    print("  %-10s %3d 次 (%.0f%%)" % (k, v, v/len(st)*100))
print()
print("=== 权重 / 门控 ===")
print("  weight: min %.0f%%  mean %.0f%%   0%% 的占比 %.1f%%" % (w.min(), w.mean(), (w==0).mean()*100))
print("  eN  max %4.0f (门限 1000)   超 1000 占比 %.1f%%" % (en.max(), (en>=1000).mean()*100))
print("  eDi max %4.0f (门限 1000)   超 1000 占比 %.1f%%" % (edi.max(), (edi>=1000).mean()*100))
