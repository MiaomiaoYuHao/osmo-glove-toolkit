import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg'], low_memory=False)
d = pd.DataFrame({k: pd.to_numeric(df[k], errors='coerce') for k in df.columns}).dropna().reset_index(drop=True)
ts = d['mag_device_time_s'].values
out = d['output_yaw_deg'].values
mag = d['mag_yaw_deg'].values

# 逐样本差分（不做 unwrap，避免解缠失败；而是对差值取环绕）
def wdiff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

dout = wdiff(out[1:], out[:-1])
dmag = wdiff(mag[1:], mag[:-1])
dt   = np.diff(ts)
ok = (np.abs(dout) < 60) & (np.abs(dmag) < 60) & (dt > 0) & (dt < 0.05)
print("参与比较的样本 %d / %d" % (ok.sum(), len(dout)))

a, b = dout[ok], dmag[ok]
print()
print("=== 变化同步性（每次采样的角度增量）===")
print("  corr(Δout, Δmag)      = %+.4f" % np.corrcoef(a,b)[0,1])
print("  回归斜率 Δmag/Δout    = %+.4f   (1.0 = 完全同步)" % (np.polyfit(a,b,1)[0]))
print("  Δout std %.3f deg   Δmag std %.3f deg" % (a.std(), b.std()))
print("  |Δout-Δmag| 中位 %.3f deg   p95 %.3f deg" % (np.median(np.abs(a-b)), np.percentile(np.abs(a-b),95)))
print()
print("=== 判定 ===")
c = np.corrcoef(a,b)[0,1]; k = np.polyfit(a,b,1)[0]
if c > 0.95 and 0.9 < k < 1.1:
    print("  两者变化【完全同步】-> 系统本身一致，那 ~180° 是参考系约定差（诊断口径问题）")
elif c > 0.7:
    print("  变化【大体同步但有偏差】-> 存在真实的航向误差/漂移成分")
else:
    print("  变化【不同步】-> 存在真实的严重问题")
