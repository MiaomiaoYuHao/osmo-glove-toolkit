import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
cols = ['quat_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg',
        'mag_state_after','mag_decision','mag_norm_lp','mag_ref_norm','yaw_deg']
df = pd.read_csv(p, usecols=lambda c: c in cols, low_memory=False)
print("行数", len(df), " 列", list(df.columns))
print()
t = df['quat_device_time_s'].astype(float)
print("设备时间范围 %.2f .. %.2f s  (时长 %.1f s)" % (t.min(), t.max(), t.max()-t.min()))

def col(name):
    if name not in df.columns: return None
    s = pd.to_numeric(df[name], errors='coerce')
    return s

out = col('output_yaw_deg'); mag = col('mag_yaw_deg'); err = col('yaw_error_deg')
st  = col('mag_state_after')

print()
print("=== 有效样本 ===")
for n,s in (('output_yaw_deg',out),('mag_yaw_deg',mag),('yaw_error_deg',err)):
    if s is not None:
        v = s.dropna()
        print("  %-16s 非空 %7d / %d" % (n, len(v), len(s)))

def unwrap_deg(x):
    return np.degrees(np.unwrap(np.radians(x)))

print()
print("=== 输出 yaw 漂移 ===")
v = out.dropna()
tv = t[out.notna()]
if len(v) > 100:
    y = unwrap_deg(v.values)
    print("  起 %.2f°  终 %.2f°  总变化 %.2f°" % (y[0], y[-1], y[-1]-y[0]))
    dt = tv.values[-1] - tv.values[0]
    print("  时长 %.1f s   平均速率 %.3f °/s  = %.2f °/min" % (dt, (y[-1]-y[0])/dt, (y[-1]-y[0])/dt*60))
    # 分段看是否在漂
    N=8
    print("  分段（每段平均速率 °/min）:")
    for k in range(N):
        a,b = k*len(y)//N, (k+1)*len(y)//N
        d = (y[b-1]-y[a]) / max(tv.values[b-1]-tv.values[a],1e-9)
        print("    段%d: %+8.2f °/min   (yaw %.1f -> %.1f)" % (k+1, d*60, y[a], y[b-1]))

if err is not None:
    e = err.dropna()
    print()
    print("=== 输出yaw vs 磁航向 的差 (yaw_error_deg) ===")
    print("  均值 %+.2f°   中位 %+.2f°   std %.2f°" % (e.mean(), e.median(), e.std()))
    print("  min %+.2f°   max %+.2f°" % (e.min(), e.max()))
    for lo,hi in ((-5,5),(-10,10),(-20,20),(-30,30)):
        print("    |差| <= %2d° 占比 %.1f%%" % (hi, ((e.abs()<=hi).mean()*100)))

if st is not None:
    print()
    print("=== 磁修正状态分布 ===")
    vc = st.value_counts(dropna=False).sort_index()
    names = {0:'NO_CAL',1:'CAL',2:'ACQ',3:'LOCKED',4:'DEGRADED',5:'COAST'}
    for k,n in vc.items():
        try: kk = int(float(k))
        except Exception: kk = -1
        print("  %-9s %8d  (%.1f%%)" % (names.get(kk,'?'), n, n/len(st)*100))
