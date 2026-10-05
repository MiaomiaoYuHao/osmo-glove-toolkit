import numpy as np, sys
sys.path.insert(0, r"C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot")
import magcal as mc
z = np.load("sweep1.npz"); m = z["mag"]
b = mc.parse_blob(open("sweep1.blob","rb").read().decode("latin1").encode("latin1"))
W = b["W"][0].reshape(3,3); off = b["offset"][0]
d_true = (m-off) @ W.T

# 代理1：去均值方向
d_prox = m - m.mean(axis=0)
for name, d in (("真实(拟合W)", d_true), ("代理(去均值)", d_prox)):
    print(f"{name:14s} 覆盖 {mc.occupied_caps(d):2d}/24")

i1 = mc.cap_index(d_true); i2 = mc.cap_index(d_prox)
print(f"两者区块判定一致率: {(i1==i2).mean()*100:.2f}%")
print()
# 再看：只用最后 N 个样本时的收敛情况
for N in (500, 2000, 5000, 20000, 70555):
    print(f"  前 {N:6d} 样本 -> 真实 {mc.occupied_caps(d_true[:N]):2d}/24  代理 {mc.occupied_caps(d_prox[:N]):2d}/24")
