import numpy as np, sys, time
sys.path.insert(0, '.')
import magcal as mc

z = np.load('allsweep.npz'); m = z['mag']; q = z['quat']
print("合并样本", len(m))
print("对比基准（未加 dip 权重时）: 残差 rms 0.03515  p95 0.07147")
print()

t0 = time.time()
W, b, r, info, w = mc.robust_fit(m, q, iters=15, verbose=True)
print(f"\n耗时 {time.time()-t0:.1f}s")
print()
print("=========== 最终拟合 ===========")
print(f"  样本          {info['n']}   有效 {info['n_inlier']} ({info['n_inlier']/info['n']*100:.1f}%)")
print(f"  球面覆盖      {info['bins']}/24")
print(f"  残差 rms      {info['rms']:.6f}")
print(f"  残差 p95      {info['p95']:.6f}")
print(f"  det(W)        {info['det_W']:.8f}   对称 {np.allclose(W,W.T)}")
print(f"  三轴 span     {info['span_pct'][0]:.0f}/{info['span_pct'][1]:.0f}/{info['span_pct'][2]:.0f} %")
print(f"  offset        [{b[0]:.3f} {b[1]:.3f} {b[2]:.3f}] LSB")
print(f"  radius        {r:.4f}")
print("  W (软铁, det=1)")
for row in W:
    print(f"      [{row[0]: .7f} {row[1]: .7f} {row[2]: .7f}]")

info2 = dict(info)
blob = mc.make_blob(W, b, r)
open('final.blob','wb').write(blob)
print()
print("blob 已写出 final.blob", len(blob), "字节")

# 用 dip 一致性做最终自检（偏航无关）
Ww = np.einsum('nij,nj->ni', mc.quat_to_mat(q), (m-b)@W.T/r)
dip = np.degrees(np.arcsin(np.clip(Ww[:,2],-1,1)))
k = w > 0.5
print(f"\n自检（偏航无关）: dip {dip[k].mean():.2f} deg  std {dip[k].std():.3f} deg")
