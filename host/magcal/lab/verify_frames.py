import serial, time, sys
sys.path.insert(0, r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot')
from cobs import cobs
from utils import bowiepb as bpb
s = serial.Serial('COM3', 115200, timeout=0.1)
time.sleep(0.3); s.reset_input_buffer()
buf = bytearray(); t0 = time.time()
while time.time() - t0 < 10.0:
    d = s.read(16384)
    if d: buf.extend(d)
s.close()
total = len(buf)
print("10 秒收到 %d 字节  (%.1f KB/s)" % (total, total/10/1024))
# 按 0x00 切帧
frames = bytes(buf).split(b'\x00')
frames = [f for f in frames if f]
ok=0; bad=0; text=0; yaw=0; yaw2=0; parm=0
kinds={}
for f in frames:
    # 文本行？
    try:
        t = f.decode('ascii')
        printable = all(32 <= ord(c) < 127 or c in '\r\n\t' for c in t)
    except Exception:
        printable = False
    if printable and len(t) > 10:
        text += 1
        if t.startswith('YAW2 '): yaw2 += 1
        elif t.startswith('YAW '): yaw += 1
        elif t.startswith('PARM '): parm += 1
        continue
    try:
        p = cobs.decode(f); m = bpb.Data(); m.parse(p); ok += 1
        d = m.to_dict()
        k = 'QUAT' if isinstance(d.get('quat'), dict) else ('MAG' if isinstance(d.get('mag'), dict) else 'OTHER')
        kinds[k] = kinds.get(k,0)+1
    except Exception:
        bad += 1
print()
print("=== 帧统计（10 秒窗口）===")
print("  总帧数        %d" % len(frames))
print("  解码成功      %d   (%.1f 帧/秒)" % (ok, ok/10))
print("      QUAT %d (%.1f/s)   MAG %d (%.1f/s)" % (kinds.get('QUAT',0), kinds.get('QUAT',0)/10, kinds.get('MAG',0), kinds.get('MAG',0)/10))
print("  文本行        %d   (%.1f/s)  [YAW %d, YAW2 %d, PARM %d]" % (text, text/10, yaw, yaw2, parm))
print("  解码失败      %d   (%.1f/s)" % (bad, bad/10))
