import numpy as np, sys
sys.path.insert(0,'.')
import magcal as mc
z = np.load('sweep2.npz'); m = z['mag']
W, b, r, info, w = mc.robust_fit(m, z['quat'], iters=6, verbose=False)
d = (m - b) @ W.T
dn = d / np.linalg.norm(d, axis=1, keepdims=True)

az = np.degrees(np.arctan2(dn[:,1], dn[:,0])) % 360.0
el = np.degrees(np.arcsin(np.clip(dn[:,2], -1, 1)))

print("方位角 atan2(Y,X) 分布（每 30 度一格）:")
for lo in range(0, 360, 30):
    sel = (az >= lo) & (az < lo + 30)
    n = int(sel.sum())
    bar = "#" * int(n / max(1, len(dn)) * 300)
    print("  %3d-%3d deg %7d  %s" % (lo, lo+30, n, bar))

print()
print("占据的 30 度方位角扇区数: %d / 12" % len(np.unique((az//30).astype(int))))
print("俯仰角范围: %.1f .. %.1f deg" % (el.min(), el.max()))

print()
print("各轴符号组合的样本数 (sign x, sign y):")
for sx in (-1, 1):
    for sy in (-1, 1):
        sel = (np.sign(dn[:,0]) == sx) & (np.sign(dn[:,1]) == sy)
        print("   x%s y%s : %6d" % ("+" if sx > 0 else "-", "+" if sy > 0 else "-", int(sel.sum())))
