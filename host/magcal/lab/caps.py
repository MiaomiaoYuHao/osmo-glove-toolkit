import numpy as np, sys
sys.path.insert(0, r"C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot")
import magcal as mc
z = np.load("sweep1.npz"); m = z["mag"]
blob = np.fromfile("sweep1.blob", dtype=np.uint8)
b = mc.parse_blob(blob.tobytes())
W = b["W"][0].reshape(3,3); off = b["offset"][0]; r = b["radius"][0]
d = (m-off) @ W.T
idx = mc.cap_index(d); cnt = np.bincount(idx, minlength=24)
axis = "XYZ"
print("24 个球面区块占用情况（面 = 场矢量在机体系的主导轴）:")
for k in range(24):
    ax = k//8; face = "+" if (k%8)>=4 else "-"
    o1 = (ax+1)%3; o2 = (ax+2)%3
    s1 = "+" if (k%8)%4>=2 else "-"
    s2 = "+" if (k%8)%2>=1 else "-"
    name = f"{face}{axis[ax]}  ({s1}{axis[o1]},{s2}{axis[o2]})"
    mark = f"{cnt[k]:6d}" if cnt[k] else "  --  "
    print(f"  {name:20s} {mark} {'<< 缺' if cnt[k]==0 else ''}")
print(f"\n占用 {int((cnt>0).sum())}/24，最少区块样本 {cnt[cnt>0].min()}，最多 {cnt.max()}")
