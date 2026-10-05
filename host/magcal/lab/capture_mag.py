import serial, time, numpy as np, sys, math
sys.path.insert(0, r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot')
from cobs import cobs
from utils import bowiepb as bpb

s = serial.Serial('COM3', 115200, timeout=0.05)
time.sleep(0.3); s.reset_input_buffer()
buf = bytearray(); mags = []
t0 = time.time()
while time.time() - t0 < 4.0:
    d = s.read(8192)
    if not d: continue
    buf.extend(d)
    while True:
        i = buf.find(0)
        if i < 0: break
        pkt = bytes(buf[:i]); del buf[:i+1]
        if not pkt: continue
        try:
            m = bpb.Data(); m.parse(cobs.decode(pkt))
        except Exception:
            continue
        g = m.to_dict().get('mag')
        if isinstance(g, dict):
            mags.append((g.get('x',0.0), g.get('y',0.0), g.get('z',0.0)))
s.close()

a = np.asarray(mags, dtype=float)
print("抓到 %d 个 Mag 样本" % len(a))
if len(a) > 10:
    n = np.linalg.norm(a, axis=1)
    print("  分量范围: x [%.2f, %.2f]  y [%.2f, %.2f]  z [%.2f, %.2f]" % (
        a[:,0].min(), a[:,0].max(), a[:,1].min(), a[:,1].max(), a[:,2].min(), a[:,2].max()))
    print("  |Mag| 均值 %.3f   std %.3f   min %.3f   max %.3f" % (n.mean(), n.std(), n.min(), n.max()))
    print()
    print("  flash 里的 radius = 34.880")
    print("  |Mag| / radius = %.4f     <- 这就是固件眼里的 |u|" % (n.mean()/34.879955))
    print()
    print("  对比: 我采集时原始 |m| 约 34.5  (radius 34.88 -> |u| ~ 1.0)")
    if n.mean() > 42:
        print("  ==> 现在的磁场模长明显更大 —— 数据确实变了")
    elif n.mean() < 30:
        print("  ==> 现在的磁场模长明显更小")
    else:
        print("  ==> 模长和采集时接近")
