import pandas as pd, numpy as np
d = pd.read_csv('yawtrace.csv')
d = d.sort_values('ts').reset_index(drop=True)

# 静止判定：滚动 omega 低
w = 5   # 0.5s
d['omega_lp'] = d['omega'].rolling(w, min_periods=3).mean()
STILL = 8.0
d['still'] = d['omega_lp'] < STILL

# 找出连续静止段
segs=[]; cur=None
for i,r in d.iterrows():
    if r['still']:
        if cur is None: cur={'a':i,'b':i}
        else: cur['b']=i
    else:
        if cur is not None:
            if cur['b']-cur['a'] >= 8: segs.append(cur)
            cur=None
if cur is not None and cur['b']-cur['a']>=8: segs.append(cur)

print("找到 %d 个静止段（>=0.8s）" % len(segs))
print()
print("%-4s %8s %8s %8s %8s %8s %8s %8s %7s" %
      ("#","起(s)","终(s)","时长","err均值","err终值","weight","traw","outyaw"))
for k,s in enumerate(segs):
    g = d.iloc[s['a']:s['b']+1]
    print("%-4d %8.1f %8.1f %8.2f %8.1f %8.1f %7.0f%% %8.1f %8.1f" %
          (k, g['ts'].iloc[0], g['ts'].iloc[-1], g['ts'].iloc[-1]-g['ts'].iloc[0],
           g['err'].mean(), g['err'].iloc[-1], g['weight'].mean()*100,
           g['traw'].mean(), g['outyaw'].mean()))

print()
print("=== 每次静止段的 err 收敛过程（前 1.5 秒逐 0.2s）===")
for k,s in enumerate(segs):
    g = d.iloc[s['a']:min(s['a']+15, s['b']+1)]
    vals = " ".join("%+6.1f" % v for v in g['err'].values[:8])
    big = " ★大偏差" if g['err'].abs().max() > 30 else ""
    lg = " large" if (g['large']==1).any() else ""
    print("  段%-3d t=%6.1f  err: %s%s%s" % (k, d['ts'].iloc[s['a']], vals, big, lg))
