import serial, time, re, sys
s = serial.Serial('COM3', 115200, timeout=0.1)
time.sleep(0.3); s.reset_input_buffer()
buf = bytearray(); t0 = time.time()
while time.time() - t0 < 8.0:
    d = s.read(8192)
    if d: buf.extend(d)
s.close()
txt = buf.decode('utf-8','replace')
lines = [l.strip() for l in txt.split('\n') if 'YAW t=' in l]
print("抓到 YAW 行: %d 条   (总字节 %d)" % (len(lines), len(buf)))
print()
if lines:
    print("=== 前 3 行原文 ===")
    for l in lines[:3]:
        print("  " + l[:300])
    print()
    # 解析
    def num(l, key):
        m = re.search(key + r'=(-?\d+)', l)
        return int(m.group(1)) if m else None
    keys = ['t','link','innov_d10','traw_d10','target_d10','outyaw_d10','err_d10',
            'relax_x100','hold_ms','slew_dps_x10','trust_x100','large','snap','clean',
            'nis','kalman','step_d100','yawc_d10','datum_d10','blend_d10','weight_x100',
            'status','NIS_x100','yawP_x10000','R_x10000','K_x1000','sig_x1000',
            'horiz_x1000','geo_x1000','omega_dps_x10','dirlock_ms']
    print("=== 解码后 5 条 ===")
    for l in lines[:5]:
        v = {k: num(l,k) for k in keys}
        print("  t=%.2fs innov=%.1f° traw=%.2f° out=%.2f° err=%.1f° | relax=%.2f hold=%.0fms "
              "slew=%.0f°/s large=%d snap=%d clean=%d kalman=%d | K=%.3f NIS=%.2f w=%d%% st=%d" %
              ((v['t']%100000)/1000.0, (v['innov_d10'] or 0)/10, (v['traw_d10'] or 0)/10,
               (v['outyaw_d10'] or 0)/10, (v['err_d10'] or 0)/10, (v['relax_x100'] or 0)/100,
               (v['hold_ms'] or 0), (v['slew_dps_x10'] or 0)/10, v['large'] or 0,
               v['snap'] or 0, v['clean'] or 0, v['kalman'] or 0, (v['K_x1000'] or 0)/1000,
               (v['NIS_x100'] or 0)/100, v['weight_x100'] or 0, v['status'] or 0))
    # 速率
    if len(lines) >= 2:
        t_first = num(lines[0],'t'); t_last = num(lines[-1],'t')
        if t_first and t_last:
            print()
            print("  帧率 = %.1f Hz" % ((len(lines)-1)/max((t_last-t_first)/1000.0,1e-9)))
else:
    print("没抓到 YAW 行。可读文本样本：")
    for l in txt.split('\n'):
        c=''.join(ch if 32<=ord(ch)<127 else '' for ch in l).strip()
        if len(c)>25: print("   " + c[:200])
