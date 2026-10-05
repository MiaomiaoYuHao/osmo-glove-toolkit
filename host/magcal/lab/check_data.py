import numpy as np
z = np.load("sweep1.npz")
m = z["mag"]; q = z["quat"]
print("磁样本数", len(m))
print("原始磁值范围:")
for i,a in enumerate("xyz"):
    print(f"  {a}: min {m[:,i].min():10.2f}  max {m[:,i].max():10.2f}  span {np.ptp(m[:,i]):9.2f}  均值 {m[:,i].mean():8.2f}  std {m[:,i].std():7.3f}")
r = np.linalg.norm(m - m.mean(axis=0), axis=1)
print(f"去均值后模长: 均值 {r.mean():.2f}  std {r.std():.3f}  min {r.min():.2f}  max {r.max():.2f}")
print(f"逐样本模长相对波动 std/mean = {r.std()/r.mean()*100:.2f}%")
# 相邻样本的噪声（静止时）
d = np.diff(m, axis=0)
step = np.linalg.norm(d, axis=1)
print(f"相邻样本差分模长: 中位 {np.median(step):.3f}  p95 {np.percentile(step,95):.3f}")
print()
print("四元数样本", len(q), " 模长均值", np.linalg.norm(q,axis=1).mean())
