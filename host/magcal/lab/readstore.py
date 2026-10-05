import struct, numpy as np, sys
sys.path.insert(0, '.')
import magcal as mc

b = open('store_backup.bin','rb').read()
assert len(b) == 29704, len(b)
magic, ver, cnt, bsz, crc, rsv = struct.unpack_from('<IHHIII', b, 0)
print("magic=0x%08X  version=%d  count=%d  blob_size=%d" % (magic, ver, cnt, bsz))
print("crc=0x%08X  reserved=%d" % (crc, rsv))

# 校验 crc
def fnv(data):
    h = 2166136261
    for x in data:
        h ^= x; h = (h * 16777619) & 0xFFFFFFFF
    return h
calc = fnv(b[20:20+4000])
print("crc 校验: 存的 0x%08X  算的 0x%08X  %s" % (crc, calc, "OK" if calc==crc else "不一致"))
bsxc = struct.unpack_from('<I', b, 29700)[0]
bsxc_calc = fnv(b[4020:4020+25680])
print("bsx_crc: 存的 0x%08X  算的 0x%08X  %s" % (bsxc, bsxc_calc, "OK" if bsxc==bsxc_calc else "不一致"))

print()
IDX = 12
blob = b[20+IDX*200 : 20+(IDX+1)*200]
i = mc.parse_blob(blob)
print("=== blob[%d] 现状 ===" % IDX)
print("  version", i['version'], " flags 0x%02X" % i['flags'])
print("  offset  ", np.round(i['offset'][0], 4))
print("  radius  ", round(float(i['radius'][0]), 4))
W = i['W'][0].reshape(3,3)
print("  W det", round(float(np.linalg.det(W)),8))
print("  A (align) 行列式 %.5f" % np.linalg.det(i['align'].reshape(3,3)))
print("  A =\n", np.round(i['align'].reshape(3,3), 4))
print("  dip =", i['dip'])
print("  map det %.5f" % np.linalg.det(i['map'].reshape(3,3)))
print("  map_c", np.round(i['map_c'],4))
