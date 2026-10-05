import struct, numpy as np, sys
sys.path.insert(0, '.')
import magcal as mc
b = open('store_backup.bin','rb').read()
def fnv(d):
    h = 2166136261
    for x in d: h ^= x; h = (h*16777619)&0xFFFFFFFF
    return h
magic, ver, cnt, bsz, crc, rsv = struct.unpack_from('<IHHIII', b, 0)
ok_crc = (fnv(b[20:4020]) == crc)
ok_bsx = (fnv(b[4020:29700]) == struct.unpack_from('<I', b, 29700)[0])
i = mc.parse_blob(b[20+12*200:20+13*200])
print("大小 %d   magic=0x%08X ver=%d count=%d blob_size=%d" % (len(b), magic, ver, cnt, bsz))
print("crc 校验 %s    bsx_crc 校验 %s" % ("OK" if ok_crc else "失败", "OK" if ok_bsx else "失败"))
print("blob[12]: offset %s  radius %.4f  flags 0x%02X  W_det %.7f" % (
      np.round(i['offset'][0],4), float(i['radius'][0]), i['flags'],
      float(np.linalg.det(i['W'][0].reshape(3,3)))))
