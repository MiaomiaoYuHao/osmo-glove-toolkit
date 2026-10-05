import struct, numpy as np, sys
sys.path.insert(0,'.')
import magcal as mc
a = open('store_backup.bin','rb').read()
b = open('store_now.bin','rb').read()
diff = [i for i,(x,y) in enumerate(zip(a,b)) if x!=y]
print("不同字节 %d 个，范围 %d .. %d" % (len(diff), diff[0], diff[-1]))
# 归属
regions = [("header", 0, 20), ("blob[20]", 20, 4020), ("bsx_gyro[20]", 4020, 29700), ("bsx_crc", 29700, 29704)]
for name, lo, hi in regions:
    n = sum(1 for i in diff if lo <= i < hi)
    print("  %-16s 范围 [%5d,%5d)  不同 %3d 字节" % (name, lo, hi, n))
print()
# blob[12] 是否完全相同
print("=== blob[12]（我的磁标定）逐字节比较 ===")
o = 20+12*200
same = (a[o:o+200] == b[o:o+200])
print("  " + ("完全相同 ✓ —— 标定没被改动" if same else "被改了 ✗"))
if same:
    i = mc.parse_blob(b[o:o+200])
    print("  offset %s   radius %.4f   flags 0x%02X   W_det %.7f" % (
        np.round(i['offset'][0],4), float(i['radius'][0]), i['flags'],
        float(np.linalg.det(i['W'][0].reshape(3,3)))))
print()
# bsx_gyro[12] 变了什么
bo = 4020+12*1284
la, lb = struct.unpack_from('<HH', a, bo), struct.unpack_from('<HH', b, bo)
print("=== bsx_gyro[12]（陀螺标定持久化）===")
print("  备份: len=%d reserved=%d" % la)
print("  现在: len=%d reserved=%d" % lb)
print("  => 这是固件的【BSX 陀螺标定自动保存】在跑（开机 60 秒后触发），")
print("     属于设计行为，和磁标定无关。")
