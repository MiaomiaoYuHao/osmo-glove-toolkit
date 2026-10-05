import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg'], low_memory=False)
d = pd.DataFrame({k: pd.to_numeric(df[k], errors='coerce') for k in df.columns}).dropna().reset_index(drop=True)
print("样本 %d   时长 %.1f s" % (len(d), d['mag_device_time_s'].iloc[-1]-d['mag_device_time_s'].iloc[0]))

def wrap180(x):                       # 正确：映射到 [-180,180)
    return (x + 180.0) % 360.0 - 180.0

e = wrap180(d['yaw_error_deg'].values)
print()
print("=== 输出yaw - 磁航向（正确环绕）===")
print("  均值 %+8.2f   中位 %+8.2f   std %7.2f deg" % (e.mean(), np.median(e), e.std()))
print("  min %+8.2f   max %+8.2f" % (e.min(), e.max()))
for lo in (2,5,10,30,90,180):
    print("    |diff|<=%3d deg 占比 %5.1f%%" % (lo, (np.abs(e)<=lo).mean()*100))
print()
print("=== 误差的角度直方图（每 30 度一格）===")
for lo in range(-180,180,30):
    m = (e>=lo)&(e<lo+30)
    print("  %+4d~%+4d : %6d  %s" % (lo, lo+30, m.sum(), '#'*int(m.sum()/max(len(e),1)*220)))

# 直方图：输出 yaw 与磁航向各自的原始值分布
print()
print("=== 静息判定：用【输出yaw的原始变化率】===")
d['out_u'] = np.degrees(np.unwrap(np.radians(d['output_yaw_deg'].values)))
dt = d['mag_device_time_s'].diff()
d['rate'] = (d['out_u'].diff()/dt).abs()
r = d['rate']
for lo in (1,5,10,30,90,300,1000):
    print("  <= %4d deg/s : %5.1f%%" % (lo, (r<=lo).mean()*100))
print("  中位 %.2f   最大 %.0f deg/s" % (r.median(), r.max()))
