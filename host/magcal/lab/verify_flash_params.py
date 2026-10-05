import numpy as np, struct, sys
sys.path.insert(0,'.')
import magcal as mc

# 1) 从【当前 flash 读回的内容】里取参数（不是内存，不是我的 npy）
store = open('store_now.bin','rb').read()
blob  = store[20+12*200 : 20+13*200]
i = mc.parse_blob(blob)
W = i['W'][0].reshape(3,3); off = i['offset'][0]; rad = float(i['radius'][0])
print("=== flash 里的参数 ===")
print("  offset", np.round(off, 5))
print("  radius %.6f" % rad)
print("  W\n", np.round(W, 6))
print()

# 2) 拿我采集的原始数据，用这组参数算 |u|
z = np.load('allsweep.npz'); m = z['mag']
u = (m - off) @ W.T / rad
n = np.linalg.norm(u, axis=1)
print("=== 用 flash 参数 作用在我采集的原始数据上 ===")
print("  |u| 均值 %.5f   标准差 %.5f" % (n.mean(), n.std()))
print("  |u| min %.4f  max %.4f" % (n.min(), n.max()))
print("  |(|u|-1)| 均值 %.5f  (这就是固件会测到的 geo_q)" % np.abs(n-1).mean())
print("  |(|u|-1)| p95  %.5f" % np.percentile(np.abs(n-1), 95))
print()

# 3) 分时段看，确认不是某一段坏
N = len(m)
for k in range(6):
    a, b = k*N//6, (k+1)*N//6
    nn = n[a:b]
    print("  第 %d 段 (%6d 样本): |u| 均值 %.4f   std %.4f" % (k+1, b-a, nn.mean(), nn.std()))
