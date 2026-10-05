import numpy as np, sys
sys.path.insert(0,'.')
import magcal as mc

def fw_bin(d):
    # 完全照抄固件 mf_cal_bin
    ax = 0; m = abs(d[0])
    if abs(d[1]) > m: m = abs(d[1]); ax = 1
    if abs(d[2]) > m: ax = 2
    o1 = (ax + 1) % 3; o2 = (ax + 2) % 3
    b = ax * 8
    if d[ax] > 0: b += 4
    if d[o1] > 0: b += 2
    if d[o2] > 0: b += 1
    return b

rng = np.random.default_rng(0)
v = rng.normal(size=(20000,3)); v /= np.linalg.norm(v,axis=1,keepdims=True)
mine = mc.cap_index(v)
ref = np.array([fw_bin(x) for x in v])
print("与固件算法一致率: %.4f%%" % ((mine==ref).mean()*100))
bad = np.where(mine!=ref)[0]
print("不一致样本数:", len(bad))
