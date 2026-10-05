import struct, hashlib, numpy as np, sys
sys.path.insert(0, '.')
import magcal as mc

def fnv(d):
    h = 2166136261
    for x in d: h ^= x; h = (h*16777619)&0xFFFFFFFF
    return h

for name in ('store_backup.bin', 'store_now.bin'):
    b = open(name,'rb').read()
    magic, ver, cnt, bsz, crc, rsv = struct.unpack_from('<IHHIII', b, 0)
    i = mc.parse_blob(b[20+12*200:20+13*200])
    print("%-20s len=%d sha=%s" % (name, len(b), hashlib.sha256(b).hexdigest()[:16]))
    print("    magic=0x%08X ver=%d count=%d blob_size=%d" % (magic, ver, cnt, bsz))
    print("    crc 0x%08X (%s)   bsx_crc 0x%08X (%s)" % (
        crc, "OK" if fnv(b[20:4020])==crc else "坏",
        struct.unpack_from('<I', b, 29700)[0],
        "OK" if fnv(b[4020:29700])==struct.unpack_from('<I', b, 29700)[0] else "坏"))
    print("    blob[12]: offset %s  radius %.4f  flags 0x%02X  W_det %.7f" % (
        np.round(i['offset'][0],4), float(i['radius'][0]), i['flags'],
        float(np.linalg.det(i['W'][0].reshape(3,3)))))
    print()

a = open('store_backup.bin','rb').read()
c = open('store_now.bin','rb').read()
print("==> 和之前烧进去的备份" + (" 逐字节一致 ✓" if a==c else " 不一致 ✗"))
if a != c:
    d = [i for i,(x,y) in enumerate(zip(a,c)) if x!=y]
    print("    不同字节数 %d，首个位置 %s" % (len(d), d[:10]))
