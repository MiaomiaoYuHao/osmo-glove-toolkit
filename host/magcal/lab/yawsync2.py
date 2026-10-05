import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg'], low_memory=False)
d = pd.DataFrame({k: pd.to_numeric(df[k], errors='coerce') for k in df.columns}).dropna().reset_index(drop=True)
d = d.rename(columns={'mag_device_time_s':'ts'}).sort_values('ts').reset_index(drop=True)

# 0.5 秒窗口
win = 0.5
d['bucket'] = (d['ts'] // win).astype(int)
g = d.groupby('bucket').agg(out_u=('output_yaw_deg', lambda s: s.iloc[0]),
                            mag_u=('mag_yaw_deg', lambda s: s.iloc[0]),
                            n=('ts','size')).reset_index()
g['t'] = g['bucket']*win
# 逐窗增量（环绕）
def wd(a,b): return (a-b+180.0)%360.0-180.0
g['d_out'] = wd(g['out_u'], g['out_u'].shift())
g['d_mag'] = wd(g['mag_u'], g['mag_u'].shift())
g = g.dropna(subset=['d_out','d_mag'])
g = g[(g['d_out'].abs()<90)&(g['d_mag'].abs()<90)]      # 排除快速转动带来的混叠
print("窗口数 %d (%0.1f 秒/窗)" % (len(g), win))
print()
print("=== 0.5 秒窗口的增量同步性 ===")
a,b = g['d_out'].values, g['d_mag'].values
print("  corr = %+.4f" % np.corrcoef(a,b)[0,1])
print("  斜率 Δmag/Δout = %+.4f" % np.polyfit(a,b,1)[0])
print("  |Δout-Δmag| 中位 %.3f deg   p95 %.3f deg" % (np.median(np.abs(a-b)), np.percentile(np.abs(a-b),95)))
print()
# 恒定偏置判定：误差的稳定性
e = (d['output_yaw_deg'].values - d['mag_yaw_deg'].values)
e = (e+180)%360-180
print("=== 输出yaw - 磁航向 的稳定性 ===")
print("  用圆周统计：")
ang = np.radians(e)
R = np.hypot(np.cos(ang).mean(), np.sin(ang).mean())
mu = np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()))
print("    平均方向 %+.2f deg   集中度 R=%.4f" % (mu, R))
print("    等效标准差 %.1f deg  (R 越接近1 越说明偏置恒定)" % np.degrees(np.sqrt(-2*np.log(max(R,1e-9)))))
print()
# 按大块看平均方向是否漂
N=12; idx=np.linspace(0,len(d)-1,N+1).astype(int)
print("  分块平均方向（若恒定则都在同一值）:")
for k in range(N):
    a_,b_ = idx[k], idx[k+1]
    if b_<=a_: continue
    seg = e[a_:b_]
    an = np.radians(seg)
    mm = np.degrees(np.arctan2(np.sin(an).mean(), np.cos(an).mean()))
    RR = np.hypot(np.cos(an).mean(), np.sin(an).mean())
    print("     t=%6.1f~%6.1f  n=%5d  平均方向 %+8.2f deg  R=%.3f" %
          (d['ts'].iloc[a_], d['ts'].iloc[b_-1], len(seg), mm, RR))
