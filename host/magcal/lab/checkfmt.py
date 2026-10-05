import re
src = open(r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\firmware\BowieGlove_Attitude\Core\Src\glove\glove.c',
           encoding='utf-8', errors='replace').read()

def check(name):
    i = src.find('printf("' + name + ' ')
    if i < 0: print(f"{name}: 找不到"); return
    # 找 printf( ... ); 整体（按括号配对）
    j = src.index('(', i + 6)
    depth = 0; k = j
    while k < len(src):
        if src[k] == '(': depth += 1
        elif src[k] == ')':
            depth -= 1
            if depth == 0: break
        k += 1
    body = src[j+1:k]
    # 分离第一个参数（格式串）与其余
    depth = 0; instr = False; esc = False; cut = -1
    for idx,ch in enumerate(body):
        if esc: esc = False; continue
        if ch == '\\': esc = True; continue
        if ch == '"': instr = not instr; continue
        if instr: continue
        if ch in '(': depth += 1
        elif ch in ')': depth -= 1
        elif ch == ',' and depth == 0:
            cut = idx; break
    fmt = body[:cut]
    rest = body[cut+1:]
    # 统计格式串里的 %
    spec = len(re.findall(r'%(?:[-+0-9.#]*)(?:l|ll|h)?[diuxX]', fmt))
    # 统计 rest 里的顶层逗号
    depth = 0; instr = False; esc = False; nargs = 0; cur = ''
    for ch in rest:
        if esc: cur += ch; esc = False; continue
        if ch == '\\': cur += ch; esc = True; continue
        if ch == '"': instr = not instr; cur += ch; continue
        if instr: cur += ch; continue
        if ch in '(': depth += 1
        elif ch in ')': depth -= 1
        elif ch == ',' and depth == 0:
            if cur.strip(): nargs += 1
            cur = ''
            continue
        cur += ch
    if cur.strip(): nargs += 1
    print("%-6s 格式符 %2d   实参 %2d   %s" % (name, spec, nargs, "✓ 匹配" if spec==nargs else "✗ 不匹配"))

for n in ['PARM','YAW2','YAW','YF']:
    check(n)

