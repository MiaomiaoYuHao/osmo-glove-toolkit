import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in
                 ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg'],
                 low_memory=False)
d = pd.DataFrame({
    'ts':  pd.to_numeric(df['mag_device_time_s'], errors='coerce'),
    'out': pd.to_numeric(df['output_yaw_deg'], errors='coerce'),
    'mag': pd.to_numeric(df['mag_yaw_deg'], errors='coerce'),
    'err': pd.to_numeric(df['yaw_error_deg'], errors='coerce'),
}).dropna().reset_index(drop=True)
print("有效样本 %d   设备时间 %.1f .. %.1f s  时长 %.1f s" %
      (len(d), d['ts'].iloc[0], d['ts'].iloc[-1], d['ts'].iloc[-1]-d['ts'].iloc[0]))

def unw(x): return np.degrees(np.unwrap(np.radians(x.values)))
d['out_u'] = unw(d['out'])
dt = d['ts'].diff()
d['rate'] = (d['out_u'].diff() / dt).abs()
med_dt = float(np.nanmedian(dt[dt>0]))
w = max(int(0.5/med_dt), 10)
d['rate_lp'] = d['rate'].rolling(w, min_periods=5).mean()
print("中位间隔 %.2f ms   滚动窗口 %d 样本 (%.2f s)" % (med_dt*1000, w, w*med_dt))

def wrap180(x): return np.degrees(np.arcsin(np.sin(np.radians(x))))

print()
print("=== 整体角速度分布（输出 yaw 的变化率）===")
r = d['rate'].dropna()
for lo in (1,5,10,30,90,300):
    print("  <= %3d deg/s : %5.1f%%" % (lo, (r<=lo).mean()*100))
print("  最大 %.0f deg/s   中位 %.2f deg/s" % (r.max(), r.median()))

for STILL in (5.0, 15.0, 40.0):
    st = d[d['rate_lp'] < STILL]
    if len(st) < 50: 
        print("\n静止段 (<%.0f deg/s): 样本太少 (%d)" % (STILL, len(st))); continue
    print()
    print("=== 静止段 (滚动角速度 < %.0f deg/s): %d 样本 (%.1f%%) ===" % (STILL, len(st), len(st)/len(d)*100))
    ew = wrap180(st['err'].values)
    print("  静止时  输出yaw - 磁航向:")
    print("     均值 %+7.2f   中位 %+7.2f   std %6.2f deg" % (ew.mean(), np.median(ew), ew.std()))
    print("     p5/p95 %+7.2f / %+7.2f   min %+7.2f  max %+7.2f" %
          (np.percentile(ew,5), np.percentile(ew,95), ew.min(), ew.max()))
    for lo in (2,5,10):
        print("       |diff|<=%2d deg 占比 %.1f%%" % (lo, (np.abs(ew)<=lo).mean()*100))
    print("  分 10 块看误差是否在爬:")
    N=10; idx = np.linspace(0, len(st)-1, N+1).astype(int); prev=None
    for k in range(N):
        a,b = idx[k], idx[k+1]
        if b<=a: continue
        seg = st.iloc[a:b]
        med = float(np.median(wrap180(seg['err'].values)))
        dd = "" if prev is None else "  (%+.2f)" % (med-prev)
        prev = med
        print("     t=%6.1f~%6.1f s  n=%5d  err中位 %+8.2f%s" % (seg['ts'].iloc[0], seg['ts'].iloc[-1], len(seg), med, dd))
