import pandas as pd, numpy as np
p = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\recordings\20260921_222821_pose_monitor_trace\analysis.csv'
df = pd.read_csv(p, usecols=lambda c: c in
                 ['mag_device_time_s','output_yaw_deg','mag_yaw_deg','yaw_error_deg',
                  'mag_norm_lp','mag_ref_norm','mag_current_norm','mag_quality','mag_state_after'],
                 low_memory=False)
d = pd.DataFrame({k: pd.to_numeric(df[k], errors='coerce') for k in df.columns}).dropna(subset=['mag_device_time_s','yaw_error_deg']).reset_index(drop=True)
d['err_w'] = np.degrees(np.arcsin(np.sin(np.radians(d['yaw_error_deg']))))

# 三个窗口：正常 / 突跳1 / 突跳2 / 正常尾
wins = [('正常 A',176,236),('突跳 1',259,286),('正常 B',305,353),('突跳 2',353,377),('正常 C',377,415)]
print("%-9s %8s %10s %10s %10s %10s %8s" % ("窗口","样本","err中位","norm_lp","ref_norm","比值","state"))
for name,a,b in wins:
    s = d[(d['mag_device_time_s']>=a)&(d['mag_device_time_s']<b)]
    if len(s)<20: continue
    nl = s['mag_norm_lp'].median(); rf = s['mag_ref_norm'].median()
    ratio = nl/rf if rf and rf>0 else float('nan')
    st = s['mag_state_after'].mode()
    stv = st.iloc[0] if len(st) else float('nan')
    print("%-9s %8d %+10.2f %10.3f %10.3f %10.3f %8.0f" %
          (name, len(s), np.median(s['err_w']), nl, rf, ratio, stv))
print()
names={0:'NO_CAL',1:'CAL',2:'ACQ',3:'LOCKED',4:'DEGRADED',5:'COAST'}
print("=== mag_state_after 分布（整段）===")
vc = d['mag_state_after'].value_counts().sort_index()
for k,n in vc.items():
    print("  %-9s %7d  (%.1f%%)" % (names.get(int(k),'?'), n, n/len(d)*100))
print()
print("=== 突跳段的模长偏离 ===")
for name,a,b in [('突跳 1',259,286),('突跳 2',353,377)]:
    s = d[(d['mag_device_time_s']>=a)&(d['mag_device_time_s']<b)]
    rf = s['mag_ref_norm'].median()
    dev = (s['mag_norm_lp']/rf - 1.0)*100
    print("  %s: 模长偏离 ref 中位 %+.1f%%   p95 %+.1f%%" % (name, dev.median(), dev.abs().quantile(0.95)))
for name,a,b in [('正常 A',176,236),('正常 C',377,415)]:
    s = d[(d['mag_device_time_s']>=a)&(d['mag_device_time_s']<b)]
    rf = s['mag_ref_norm'].median()
    dev = (s['mag_norm_lp']/rf - 1.0)*100
    print("  %s: 模长偏离 ref 中位 %+.1f%%   p95 %+.1f%%" % (name, dev.median(), dev.abs().quantile(0.95)))
