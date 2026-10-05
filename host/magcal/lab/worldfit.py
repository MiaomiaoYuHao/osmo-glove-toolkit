import numpy as np, sys, time
sys.path.insert(0, '.')
import magcal as mc

z = np.load('allsweep.npz'); m = z['mag']; q = z['quat']
print("样本", len(m))

t0 = time.time()
W0, b0, r0, info0, w0 = mc.robust_fit(m, q, iters=12, verbose=False)
print(f"\n[第 1 步] 椭球拟合 (耗时 {time.time()-t0:.1f}s)")
print(f"  覆盖 {info0['bins']}/24   残差 rms {info0['rms']:.5f}  p95 {info0['p95']:.5f}")

t0 = time.time()
res = mc.refine_world(m, q, (W0, b0, r0), w0, verbose=True)
print(f"  (耗时 {time.time()-t0:.1f}s)")

if res is not None:
    W1, b1, r1 = res
    d1 = (m - b1) @ W1.T
    r_ = np.linalg.norm(d1, axis=1) - r1
    r_ = r_ / r1
    keep = w0 > 0.5
    print()
    print("[第 2 步] 世界系精修后的椭球")
    print(f"  覆盖 {len(np.unique(mc.cap_index(d1)))}/24")
    print(f"  |u| 残差 rms {np.sqrt(np.mean(r_[keep]**2)):.6f}  p95 {np.percentile(np.abs(r_[keep]),95):.6f}")
    print()
    print("  椭球参数变化:")
    print(f"    offset  {np.round(b0,3)}  ->  {np.round(b1,3)}   (差 {np.linalg.norm(b1-b0):.4f} LSB)")
    print(f"    radius  {r0:.4f}  ->  {r1:.4f}")
    print(f"    W 最大元素变化 {np.max(np.abs(W1-W0)):.6f}")
    print(f"    det(W) {np.linalg.det(W1):.8f}   对称 {np.allclose(W1,W1.T)}")
    np.save('allsweep_Wworld.npy', W1); np.save('allsweep_bworld.npy', b1)
    open('allsweep_rworld.txt','w').write(repr(r1))
    open('allsweep.blob','wb').write(mc.make_blob(W1, b1, r1))
    print("  已写出 allsweep.blob")
