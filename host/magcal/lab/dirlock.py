import pandas as pd, numpy as np
d = pd.read_csv('yawtrace.csv').sort_values('ts').reset_index(drop=True)
print("=== dir_lock 活跃度 ===")
act = d['dirlock_ms'] > 0
print("  活跃样本 %d / %d  (%.1f%%)" % (act.sum(), len(d), act.mean()*100))
print("  活跃时剩余时间: 中位 %.0fms  均值 %.0fms" % (d.loc[act,'dirlock_ms'].median(), d.loc[act,'dirlock_ms'].mean()))
print()
print("=== trust 与 slew 的关系（证明 dir_lock 在压速度）===")
for tl,tm in [('dir_lock=0','dirlock_ms<=0'),('dir_lock>0','dirlock_ms>0')]:
    g = d.query(tm)
    if len(g)==0: continue
    print("  %-12s n=%4d  trust均值 %.3f  slew中位 %.1f°/s  slew=91.6(最慢)占比 %.0f%%" %
          (tl, len(g), g['trust'].mean(), g['slew'].median(), (np.abs(g['slew']-91.6)<1).mean()*100))
print()
print("=== 大修正路径的实际档位分布 ===")
L = d[d['large']==1]
print("  large=1 样本 %d" % len(L))
print("    relax 分布:", dict(pd.cut(L['relax'], [0.99,1.01,1.26,1.51], labels=['1.00(原版)','1.01~1.25','1.26~1.50']).value_counts()))
print("    hold 分布:", dict(pd.cut(L['hold_ms'], [-1,1,100,201], labels=['0(不冻结)','1~100ms','101~200ms']).value_counts()))
print("    snap 次数:", int(L['snap'].sum()))
print("    clean=1 占比: %.0f%%" % ((L['clean']==1).mean()*100))
print()
print("=== innov（循环欠账）分布 —— 决定档位的就是它 ===")
for lo,hi,lab in [(0,5,'<5°  → relax=1.0 hold=200ms'),(5,15,'5~15° → relax 上升 hold 缩短'),(15,20,'15~20°'),(20,999,'>=20° → 应全速回零')]:
    m=(d['innov']>=lo)&(d['innov']<hi)
    if m.sum()==0: continue
    sub=d[m]
    print("  %-30s n=%4d (%.0f%%)  snap=%d  relax中位%.2f hold中位%.0fms slew中位%.1f" %
          (lab, m.sum(), m.mean()*100, int(sub['snap'].sum()),
           sub['relax'].median(), sub['hold_ms'].median(), sub['slew'].median()))
