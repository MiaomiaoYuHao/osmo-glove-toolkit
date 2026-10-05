import serial, time, re, numpy as np
s = serial.Serial('COM3', 115200, timeout=0.1)
time.sleep(0.3); s.reset_input_buffer()
buf = bytearray(); t0 = time.time()
while time.time() - t0 < 12.0:
    d = s.read(8192)
    if d: buf.extend(d)
s.close()
txt = buf.decode('utf-8','replace')
lines = [l.strip() for l in txt.split('\n') if 'YAW t=' in l]
def num(l,k):
    m = re.search(k+r'=(-?\d+)', l); return int(m.group(1)) if m else 0
keys = ['innov_d10','traw_d10','outyaw_d10','err_d10','relax_x100','hold_ms',
        'slew_dps_x10','trust_x100','large','snap','clean','nis','kalman','step_d100',
        'yawc_d10','datum_d10','blend_d10','weight_x100','status','NIS_x100',
        'yawP_x10000','R_x10000','K_x1000','sig_x1000','horiz_x1000','geo_x1000',
        'omega_dps_x10','dirlock_ms']
D = {k: np.array([num(l,k) for l in lines], dtype=float) for k in keys}
print("样本 %d 条" % len(lines))
print()
def show(name, arr, scale=1.0, unit=''):
    print("  %-14s min %9.2f  max %9.2f  mean %9.2f  std %8.2f %s" %
          (name, arr.min()/scale, arr.max()/scale, arr.mean()/scale, arr.std()/scale, unit))
print("=== 权威量 ===")
show("traw", D['traw_d10'], 10, 'deg')
show("out_yaw", D['outyaw_d10'], 10, 'deg')
show("err=traw-outyaw", D['err_d10'], 10, 'deg')
show("datum", D['datum_d10'], 10, 'deg')
show("yawc", D['yawc_d10'], 10, 'deg')
print()
print("=== 修正环 ===")
show("innov(欠账)", D['innov_d10'], 10, 'deg')
show("K", D['K_x1000'], 1000)
show("yawP", D['yawP_x10000'], 10000)
show("R", D['R_x10000'], 10000)
show("NIS", D['NIS_x100'], 100)
show("sig", D['sig_x1000'], 1000)
show("horiz", D['horiz_x1000'], 1000)
show("geo", D['geo_x1000'], 1000)
print()
print("=== 门控/状态 ===")
for k,name in [('weight_x100','weight%'),('status','status'),('large','large'),
               ('snap','snap'),('clean','clean'),('kalman','kalman'),('nis','nis_flag'),
               ('dirlock_ms','dirlock_ms')]:
    v = D[k]
    print("  %-14s 取值: %s" % (name, sorted(set(v.astype(int).tolist()))[:10]))
print()
print("=== 大修正路径有没有被触发 ===")
print("  large=1 的样本: %d / %d" % ((D['large']==1).sum(), len(lines)))
print("  snap =1 的样本: %d" % (D['snap']==1).sum())
print("  clean=1 的样本: %d" % (D['clean']==1).sum())
print("  weight=0 的样本: %d (%.0f%%)" % ((D['weight_x100']==0).sum(), (D['weight_x100']==0).mean()*100))
