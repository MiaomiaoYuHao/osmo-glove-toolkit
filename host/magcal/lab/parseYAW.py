import json, re, numpy as np, pandas as pd
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_225025_pose_monitor_trace\firmware_diagnostics.jsonl'
FIELDS = {   # 输出列名 -> (jsonl 里的原始键, 缩放)
 'innov':('innov_d10',10), 'traw':('traw_d10',10), 'target':('target_d10',10),
 'outyaw':('outyaw_d10',10), 'err':('err_d10',10), 'relax':('relax_x100',100),
 'hold_ms':('hold_ms',1), 'slew':('slew_dps_x10',10), 'trust':('trust_x100',100),
 'large':('large',1), 'snap':('snap',1), 'clean':('clean',1), 'nisflag':('nis',1),
 'kalman':('kalman',1), 'step':('step_d100',100), 'yawc':('yawc_d10',10),
 'datum':('datum_d10',10), 'blend':('blend_d10',10), 'weight':('weight_x100',100),
 'status':('status',1), 'NIS':('NIS_x100',100), 'yawP':('yawP_x10000',10000),
 'R':('R_x10000',10000), 'K':('K_x1000',1000), 'sig':('sig_x1000',1000),
 'horiz':('horiz_x1000',1000), 'geo':('geo_x1000',1000), 'omega':('omega_dps_x10',10),
 'dirlock_ms':('dirlock_ms',1), 'tms':('t',1),
}
rows=[]
with open(p, encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        if 'YAW t=' not in ln: continue
        try: o=json.loads(ln)
        except Exception: continue
        line=o.get('line','')
        r={'wall':o.get('rx_wall_time')}
        for out,(key,sc) in FIELDS.items():
            m=re.search(r'\b'+key+r'=(-?\d+)', line)
            r[out]=(int(m.group(1))/sc) if m else np.nan
        rows.append(r)
d=pd.DataFrame(rows)
d['ts']=d['tms']/1000.0 - d['tms'].iloc[0]/1000.0
d=d.sort_values('ts').reset_index(drop=True)
print("样本 %d   时长 %.1f s   中位间隔 %.3f s" % (len(d), d['ts'].iloc[-1], np.median(np.diff(d['ts']))))
print()
print("=== 总体 ===")
for k in ['err','innov','weight','geo','traw','outyaw','yawc','datum','K','NIS','omega','dirlock_ms']:
    print("  %-10s min %9.2f  max %9.2f  mean %9.2f" % (k, d[k].min(), d[k].max(), d[k].mean()))
print()
print("=== 大修正路径 ===")
for k in ['large','snap','clean','kalman']:
    print("  %-7s = 1 : %5d / %d  (%.1f%%)" % (k,(d[k]==1).sum(),len(d),(d[k]==1).mean()*100))
print()
print("  weight 分布:")
for lo,hi in [(0,0.001),(0.001,50),(50,99.9),(99.9,100.1)]:
    m=(d['weight']>=lo)&(d['weight']<=hi)
    print("    %6.1f~%6.1f%% : %5d (%.1f%%)" % (lo,hi,m.sum(),m.mean()*100))
print()
print("  status:", dict(d['status'].value_counts().sort_index()))
print("  relax 取值:", sorted(set(np.round(d['relax'],2))))
print("  slew  取值:", sorted(set(np.round(d['slew'],1)))[:12], "...")
print("  hold_ms 取值:", sorted(set(np.round(d['hold_ms'],0)))[:12], "...")
d.to_csv('yawtrace.csv', index=False)
print()
print("已存 yawtrace.csv")
