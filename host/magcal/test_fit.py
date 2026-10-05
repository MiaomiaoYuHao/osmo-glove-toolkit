import numpy as np
import magcal as mc
rng = np.random.default_rng(11)

# 真值：W 对称 det=1，radius 在 LSB 量级（和固件一致）
b_true = np.array([120.0, -340.0, 85.0])
P = rng.normal(size=(3,3)); P = P.T@P + np.eye(3)*2
S0 = mc.sym_sqrt(P); W_true = S0/np.cbrt(np.linalg.det(S0))
r_true = 2000.0
print("真值 W 对称?", np.allclose(W_true,W_true.T), " det=", round(float(np.linalg.det(W_true)),8), " radius=", r_true)

N = 5000
u = rng.normal(size=(N,3)); u /= np.linalg.norm(u,axis=1,keepdims=True)
m = (np.linalg.solve(W_true, u.T).T)*r_true + b_true
m += rng.normal(scale=5.0, size=m.shape)
k = int(N*0.12); idx = rng.choice(N,k,replace=False)
m[idx] += rng.normal(scale=250.0, size=(k,3))
q = rng.normal(size=(N,4)); q/=np.linalg.norm(q,axis=1,keepdims=True)
print("原始跨度", np.round(np.ptp(m, axis=0),0))

W,b,r,info,w = mc.robust_fit(m, q, iters=15)
print()
print("=== 与真值对比 ===")
print("offset 误差    ", np.round(b-b_true,3), " LSB")
print("radius 误差    ", round(r-r_true,4), f"({(r-r_true)/r_true*100:.4f}%)")
print("W 最大元素误差 ", round(float(np.max(np.abs(W-W_true))),8))
print("W 对称?", np.allclose(W,W.T), " det=", round(float(np.linalg.det(W)),8))
print("有效样本       ", round(info['w_eff']), "/", N, "(真离群", k, ")")

uu_fit  = (m-b)@W.T/r
uu_true = (m-b_true)@W_true.T/r_true
ang = np.degrees(np.arccos(np.clip(np.sum(uu_fit*uu_true,axis=1),-1,1)))
print("u 方向误差: 中位", round(float(np.median(ang)),5), "deg   95%", round(float(np.percentile(ang,95)),5), "deg")

blob = mc.make_blob(W,b,r); back = mc.parse_blob(blob)
print("blob 大小", len(blob), " 回读 det", round(float(np.linalg.det(back['W'][0].reshape(3,3))),8), " radius", round(float(back['radius'][0]),4))

