# -*- coding: utf-8 -*-
c = r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\firmware\BowieGlove_Attitude\Core\Src\glove\glove.c'
lines = open(c, encoding='utf-8').read().split('\n')
a = next(i for i,l in enumerate(lines) if l.startswith('#if MAG_YAW_FAST_HZ > 0'))
b = next(i for i in range(a, len(lines)) if lines[i].strip() == '#endif')
block = lines[a:b+1]
print("移除顶部 YF 块: %d .. %d" % (a+1, b+1))
del lines[a:b+1]
while a < len(lines) and lines[a].strip() == '':
    del lines[a]
q = next(i for i,l in enumerate(lines) if 'qual_x1000' in l)
end = next(i for i in range(q, min(q+140, len(lines))) if 'diag_frame_end();' in lines[i])
print("YAW2 收尾在 %d 行" % (end+1))
lines[end+1:end+1] = [''] + block
open(c, 'w', encoding='utf-8', newline='\r\n').write('\n'.join(lines))
print("OK: YF 块已移到 YAW2 之后")
