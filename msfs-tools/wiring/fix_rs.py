p = 'D:/A380/msfs-a380/wiring/gen_table2.py'
src = open(p, encoding='utf-8', newline='').read()
start = src.index('def rs(s):')
end = src.index('def one_line(')
new = (
    "def rs(s):\n"
    "    # A Rust string literal: unicode kept as is (json.dumps' escapes are not Rust).\n"
    "    s = ' '.join(str(s).split())\n"
    "    s = s.replace(chr(92), chr(92) * 2).replace('\"', chr(92) + '\"')\n"
    "    return '\"' + s + '\"'\n\n"
)
src = src[:start] + new + src[end:]
open(p, 'w', encoding='utf-8', newline='\n').write(src)
print('fixed')
