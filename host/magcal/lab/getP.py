import serial, time, sys, re
try:
    s = serial.Serial('COM3', 115200, timeout=0.3)
except Exception as e:
    print('COM3 打不开（上位机还开着？）:', e); raise SystemExit(1)
time.sleep(0.4)
s.reset_input_buffer()
s.write(b'P')          # 打印磁状态
s.flush()
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 3.0:
    d = s.read(4096)
    if d: buf.extend(d)
s.close()
txt = buf.decode('utf-8','replace')
lines = [l for l in txt.split('\n') if ('link' in l and 'status' in l) or re.match(r'^\s*\d+\s+(NO_CAL|CAL|ACQ|LOCKED|DEGRADED|COAST)', l)]
print('================= P 输出 =================')
for l in lines:
    print(l.rstrip())
if not lines:
    print('（没抓到 P 表；下面是从原始流里筛出的可读文本）')
    for l in txt.split('\n'):
        c = ''.join(ch if (32 <= ord(ch) < 127 or ch in '\t') else '' for ch in l).strip()
        if len(c) > 20: print(c)
