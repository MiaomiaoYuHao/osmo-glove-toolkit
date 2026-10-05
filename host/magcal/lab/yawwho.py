import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg'], low_memory=False)
d = pd.DataFrame({k: pd.to_numeric(df[k], errors='coerce') for k in df.columns}).dropna().reset_index(drop=True)
d['out_u'] = np.degrees(np.unwrap(np.radians(d['output_yaw_deg'].values)))
d['mag_u'] = np.degrees(np.unwrap(np.radians(d['mag_yaw_deg'].values)))

print("%-9s %9s %9s %11s %11s %11s" % ("窗口","out起→终","mag起→终","Δout","Δmag","Δerr"))
for name,a,b in [('正常 A',176,236),('突跳 1',259,286),('正常 B',305,353),('突跳 2',353,377),('正常 C',377,415)]:
    s = d[(d['mag_device_time_s']>=a)&(d['mag_device_time_s']<b)]
    if len(s)<20: continue
    do = s['out_u'].iloc[-1]-s['out_u'].iloc[0]
    dm = s['mag_u'].iloc[-1]-s['mag_u'].iloc[0]
    print("%-9s %9.1f %9.1f %+11.2f %+11.2f %+11.2f" % (name, do, dm, do, dm, do-dm))

print()
print("=== 突跳 1 的逐秒轨迹（每 5 秒一个点）===")
s = d[(d['mag_device_time_s']>=255)&(d['mag_device_time_s']<=292)]
for k in range(0, len(s), max(len(s)//14,1)):
    r = s.iloc[k]
    print("  t=%6.1f   out=%8.2f  mag=%8.2f  err=%+7.2f" % (r['mag_device_time_s'], r['output_yaw_deg'], r['mag_yaw_deg'], r['yaw_error_deg']))
print()
print("=== 突跳 2 的逐秒轨迹 ===")
s = d[(d['mag_device_time_s']>=350)&(d['mag_device_time_s']<=382)]
for k in range(0, len(s), max(len(s)//14,1)):
    r = s.iloc[k]
    print("  t=%6.1f   out=%8.2f  mag=%8.2f  err=%+7.2f" % (r['mag_device_time_s'], r['output_yaw_deg'], r['mag_yaw_deg'], r['yaw_error_deg']))
