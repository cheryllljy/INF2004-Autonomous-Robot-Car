#!/usr/bin/env python3
"""Barr-C 1.2.a helper: a trailing comment that makes a line longer than
80 columns is moved onto its own line(s) above the code it describes,
at the code's indentation. Run before clang-format, on tab-free files."""
import re, sys
LIMIT = 80

def split_trailing(line):
    """Return (code, comment_start_col) if the line has code followed by a
    comment start outside strings, else None."""
    i, n, in_s, q = 0, len(line), False, ''
    while i < n:
        c = line[i]
        if in_s:
            if c == '\\': i += 2; continue
            if c == q: in_s = False
        elif c in '"\'':
            in_s, q = True, c
        elif line.startswith('/*', i) or line.startswith('//', i):
            return i if line[:i].strip() else None
        i += 1
    return None

def fix(lines):
    out, i = [], 0
    while i < len(lines):
        line = lines[i]
        col = split_trailing(line)
        long_block = False
        if col is not None:
            comment = line[col:]
            code = line[:col].rstrip()
            block = [comment]
            j = i
            if comment.startswith('/*') and '*/' not in comment:
                # continuation lines of a trailing block comment
                while j + 1 < len(lines):
                    j += 1
                    block.append(lines[j].strip())
                    if '*/' in lines[j]:
                        break
            too_long = any(len(l) > LIMIT for l in lines[i:j + 1])
            if too_long and not code.lstrip().startswith('#if') and \
               not code.rstrip().endswith('\\'):
                ind = re.match(r'\s*', code).group(0)
                if code.lstrip().startswith('#define'):
                    ind = ''
                if comment.startswith('//'):
                    out.append(ind + comment.strip())
                elif len(block) == 1:
                    out.append(ind + comment.strip())
                else:
                    body = [b for b in block]
                    out.append(ind + body[0].rstrip())
                    for b in body[1:]:
                        b = b.strip()
                        if b.startswith('*') and not b.startswith('*/'):
                            out.append(ind + ' ' + b)
                        else:
                            out.append(ind + ' * ' + b if not b.startswith('*/') else ind + ' ' + b)
                out.append(code)
                i = j + 1
                continue
        out.append(line)
        i += 1
    return out

for f in sys.argv[1:]:
    src = open(f, encoding='utf-8', errors='surrogateescape').read()
    lines = src.split('\n')
    new = fix(lines)
    if new != lines:
        open(f, 'w', encoding='utf-8', errors='surrogateescape').write('\n'.join(new))
