import numpy as np, sys
sys.path.insert(0, '.')
import magcal as mc
z = np.load('allsweep.npz'); m = z['mag']; q = z['quat']
W, b, r, info, w = mc.robust_fit(m, q, iters=10, verbose=False)
u = (m - b) @ W.T / r
R = mc.quat_to_mat(q)
Ww = np.einsum('nij,nj->ni', R, u)          # 世界系磁场

dip = np.degrees(np.arcsin(np.clip(Ww[:,2], -1, 1)))       # 俯仰分量（偏航无关）
az  = np.degrees(np.arctan2(Ww[:,1], Ww[:,0]))             # 方位（含标定系与偏航修正）

keep = w > 0.5
print("世界系磁场分解（只看有效样本 %d 个）" % keep.sum())
print()
print("  【俯仰/dip 分量】偏航无关 —— 这是磁力计+姿态的真实一致性")
print("     均值 %.3f deg   std %.3f deg   p5..p95 %.3f .. %.3f deg" %
      (dip[keep].mean(), dip[keep].std(), np.percentile(dip[keep],5), np.percentile(dip[keep],95)))
print()
print("  【方位角】偏航相关 —— 反映 yaw_corr 的活动")
# 平滑后看慢漂移
import numpy as np
def smooth(x, k=2000):
    ker = np.ones(k)/k
    return np.convolve(x, ker, mode='valid')
azs = smooth(az)
print("     平滑后范围 %.2f .. %.2f deg   (跨度 %.2f deg)" %
      (azs.min(), azs.max(), azs.max()-azs.min()))
print("     原始 std %.2f deg" % az[keep].std())
print()
print("  结论: dip 的 std 越小，说明磁与姿态越自洽；")
print("        方位角缓慢漂移 = yaw_corr 在动（标定期间理论上应冻结）")
