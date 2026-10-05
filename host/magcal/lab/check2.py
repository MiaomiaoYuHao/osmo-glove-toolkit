import numpy as np, sys
sys.path.insert(0,'.')
import magcal as mc
z = np.load('sweep2.npz'); m = z['mag']
W, b, r, info, w = mc.robust_fit(m, z['quat'], iters=8, verbose=False)
d_true = (m - b) @ W.T
d_prox = m - m.mean(axis=0)
print()
print("代理(去均值)  覆盖", len(np.unique(mc.cap_index(d_prox))), "/24")
print("拟合椭球      覆盖", len(np.unique(mc.cap_index(d_true))), "/24")
ct = np.bincount(mc.cap_index(d_true), minlength=24)
print()
print("用拟合椭球看，每个区块的样本数:")
for k in range(24):
    tag = "  缺" if ct[k] == 0 else ""
    print("  %-14s %7d%s" % (mc.cap_name(k), ct[k], tag))
