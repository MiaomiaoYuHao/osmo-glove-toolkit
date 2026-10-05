import pandas as pd, numpy as np
d = pd.read_csv('yawtrace.csv').sort_values('ts').reset_index(drop=True)
d['omega_lp'] = d['omega'].rolling(5, min_periods=3).mean()
d['still'] = d['omega_lp'] < 8.0
segs=[]; cur=None
for i,r in d.iterrows():
    if r['still']:
        cur = {'a':i,'b':i} if cur is None else {'a':cur['a'],'b':i}
    else:
        if cur is not None and cur['b']-cur['a']>=8: segs.append(cur)
        cur=None
if cur is not None and cur['b']-cur['a']>=8: segs.append(cur)

def wd(a,b): return (a-b+180.0)%360.0-180.0
print("%-3s %7s %8s %8s %9s %8s %7s %6s %6s %6s %6s" %
      ("#","起(s)","traw","outyaw","err","weight","relax","hold","slew","large","snap"))
prev=None
for k,s in enumerate(segs):
    i = s['b']                 # 段末（已经停稳）
    r = d.iloc[i]
    tr, oy = r['traw'], r['outyaw']
    e = wd(tr, oy)
    ch = "" if prev is None else "  Δerr=%+.1f" % wd(e, prev)
    prev = e
    print("%-3d %7.1f %8.2f %8.2f %+9.2f %7.0f%% %6.2f %6.0f %6.0f %6.0f %6.0f%s" %
          (k, r['ts'], tr, oy, e, r['weight']*100, r['relax'], r['hold_ms'],
           r['slew'], r['large'], r['snap'], ch))
print()
print("=== 段末当时的完整状态 ===")
for k,s in enumerate(segs):
    r = d.iloc[s['b']]
    print("  段%-2d t=%6.1f  err=%+7.2f  innov=%6.2f  K=%.3f  NIS=%6.1f  geo=%.3f  dirlock=%.0fms  status=%d  yawc=%.1f datum=%.1f" %
          (k, r['ts'], wd(r['traw'],r['outyaw']), r['innov'], r['K'], r['NIS'], r['geo'],
           r['dirlock_ms'], r['status'], r['yawc'], r['datum']))
