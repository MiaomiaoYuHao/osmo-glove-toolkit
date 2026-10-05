import struct, sys, numpy as np
sys.path.insert(0,'.')
import magcal as mc
b = open('store_verify.bin','rb').read()
nz = [i for i,x in enumerate(b) if x != 0xFF]
print("非 0xFF 字节 %d 个，范围 %d .. %d" % (len(nz), nz[0] if nz else -1, nz[-1] if nz else -1))
# 按区域归属
regions = [("header",0,20), ("blob[0]",20,220)] + [("blob[%d]"%k, 20+k*200, 20+(k+1)*200) for k in (1,2,11,12)]
for name,lo,hi in [("header",0,20)] + [("blob[%d]"%k, 20+k*200, 20+(k+1)*200) for k in range(20)]:
    n = sum(1 for i in nz if lo<=i<hi)
    if n: print("  %-12s [%5d,%5d)  非FF %3d 字节" % (name,lo,hi,n))
print()
magic, ver, cnt, bsz, crc, rsv = struct.unpack_from('<IHHIII', b, 0)
print("header: magic=0x%08X version=%d count=%d blob_size=%d crc=0x%08X" % (magic,ver,cnt,bsz,crc))
def fnv(d):
    h=2166136261
    for x in d: h^=x; h=(h*16777619)&0xFFFFFFFF
    return h
print("crc 校验:", "OK" if fnv(b[20:4020])==crc else "不匹配(算得 0x%08X)"%fnv(b[20:4020]))
for k in range(20):
    o = 20+k*200
    if all(x==0xFF for x in b[o:o+200]): continue
    try:
        i = mc.parse_blob(b[o:o+200])
        print("  blob[%d]: ver=%d flags=0x%02X offset=%s radius=%.3f W_det=%.6f" % (
            k, i['version'], i['flags'], np.round(i['offset'][0],4),
            float(i['radius'][0]), float(np.linalg.det(i['W'][0].reshape(3,3)))))
    except Exception as e:
        print("  blob[%d]: 解析失败 %s" % (k,e))
