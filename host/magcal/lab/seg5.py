import pandas as pd, numpy as np
d = pd.read_csv('yawtrace.csv').sort_values('ts').reset_index(drop=True)

def wd(a,b): return (a-b+180.0)%360.0-180.0
# 段5 出现前后的细节
print("=== 段5 前后（t=100~160s）逐样本，每 0.5s 一行 ===")
g = d[(d['ts']>=100)&(d['ts']<=160)].reset_index(drop=True)
for k in range(0, len(g), 5):
    r = g.iloc[k]
    print("  t=%6.1f omega=%6.1f traw=%8.2f out=%8.2f err=%+7.1f innov=%5.2f "
          "large=%d clean=%d snap=%d relax=%.2f hold=%3.0f slew=%5.1f K=%.3f w=%3.0f%% st=%d dl=%4.0f" %
          (r['ts'], r['omega'], r['traw'], r['outyaw'], wd(r['traw'],r['outyaw']),
           r['innov'], r['large'], r['clean'], r['snap'], r['relax'], r['hold_ms'],
           r['slew'], r['K'], r['weight']*100, r['status'], r['dirlock_ms']))
print()
print("=== 全部静止段里，大修正路径的使用情况 ===")
d['omega_lp'] = d['omega'].rolling(5,min_periods=3).mean()
d['still'] = d['omega_lp'] < 8.0
segs=[]; cur=None
for i,r in d.iterrows():
    if r['still']: cur = {'a':i,'b':i} if cur is None else {'a':cur['a'],'b':i}
    else:
        if cur is not None and cur['b']-cur['a']>=8: segs.append(cur)
        cur=None
if cur is not None and cur['b']-cur['a']>=8: segs.append(cur)
for k,s in enumerate(segs):
    g = d.iloc[s['a']:s['b']+1]
    print("  段%-2d  n=%3d  large=%3d clean=%3d snap=%d  weight均值%3.0f%%  dirlock均值%4.0fms  omega均值%6.1f" %
          (k, len(g), (g['large']==1).sum(), (g['clean']==1).sum(), (g['snap']==1).sum(),
           g['weight'].mean()*100, g['dirlock_ms'].mean(), g['omega'].mean()))
