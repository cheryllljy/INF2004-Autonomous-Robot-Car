#!/usr/bin/env python3
"""Barr-C 2018 mechanical fixes on C files, code only (comments, strings
and preprocessor conditionals are left alone):
  1.8  LOCAL/EXPORT/IMPORT (T-Kernel macros) -> static / (nothing) / extern
  5.2  plain C integer types -> <stdint.h> fixed-width types
       (bit-field declarations keep 'unsigned int', which C requires)
  8.6  equality tests with a constant on the right -> constant on the left
Usage: barr_fix.py FILE...   (edits in place, prints a count per file)"""
import re, sys

TOK = re.compile(r'''//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|'(?:\\.|[^'\\\n])*\'''', re.S)

TYPES = [
    (r'\bunsigned\s+long\s+long(?:\s+int)?\b', 'uint64_t'),
    (r'\b(?:signed\s+)?long\s+long(?:\s+int)?\b', 'int64_t'),
    (r'\bunsigned\s+long(?:\s+int)?\b', 'uint32_t'),
    (r'\bunsigned\s+short(?:\s+int)?\b', 'uint16_t'),
    (r'\bunsigned\s+char\b', 'uint8_t'),
    (r'\bunsigned\s+int\b', 'uint32_t'),
    (r'\bunsigned\b(?!\s*(?:char|short|long|int)\b)', 'uint32_t'),
    (r'\bsigned\s+char\b', 'int8_t'),
    (r'\b(?:signed\s+)?short(?:\s+int)?\b', 'int16_t'),
    (r'\b(?:signed\s+)?long(?:\s+int)?\b', 'int32_t'),
    (r'\bsigned\s+int\b', 'int32_t'),
    (r'\bint\b', 'int32_t'),
]
BITFIELD = re.compile(r'^\s*(?:unsigned|signed)?\s*int\s+\w+\s*:\s*\d+\s*;', re.M)

OPERAND = r'[A-Za-z_]\w*(?:(?:\.|->)[A-Za-z_]\w*|\[[^\[\]\n]*\])*'
CONST = r'(?:[A-Z_][A-Z0-9_]*|0[xX][0-9A-Fa-f]+[uUlL]*|\d+[uUlL]*)'
EQ = re.compile(r'(\(|&&|\|\||\breturn\b|[^=!<>&|]=|,|\?)(\s*)(' + OPERAND +
                r')(\s*)(==|!=)(\s*)(' + CONST + r')(?=\s*(?:\)|&&|\|\||;|\?|,|:))')

def is_caps(s):
    return re.fullmatch(r'[A-Z_][A-Z0-9_]*', s) is not None

def fix_code(code):
    n = 0
    # 1.8 T-Kernel storage-class macros
    code, k = re.subn(r'\bLOCAL\b', 'static', code); n += k
    code, k = re.subn(r'\bIMPORT\b', 'extern', code); n += k
    code, k = re.subn(r'(^|\n)([ \t]*)EXPORT[ \t]+', r'\1\2', code); n += k
    # 5.2 fixed-width integers, line by line so bit-fields can be skipped
    out = []
    for line in code.split('\n'):
        if BITFIELD.match(line) or line.lstrip().startswith('#'):
            out.append(line); continue
        for pat, rep in TYPES:
            line, k = re.subn(pat, rep, line); n += k
        out.append(line)
    code = '\n'.join(out)
    # 8.6 constant on the left (skip preprocessor lines, skip caps==caps)
    out = []
    for line in code.split('\n'):
        if line.lstrip().startswith('#'):
            out.append(line); continue
        def swap(m):
            if is_caps(m.group(3)) and '.' not in m.group(3) and '->' not in m.group(3):
                return m.group(0)          # both look like constants
            return (m.group(1) + m.group(2) + m.group(7) + m.group(4) +
                    m.group(5) + m.group(6) + m.group(3))
        prev = None
        while prev != line:
            prev = line
            line, k = EQ.subn(swap, line); n += k if line != prev else 0
        out.append(line)
    return '\n'.join(out), n

def fix_file(path):
    src = open(path, encoding='utf-8', errors='surrogateescape').read()
    keep = []
    def stash(m):
        keep.append(m.group(0)); return '\x00%d\x00' % (len(keep) - 1)
    masked = TOK.sub(stash, src)
    fixed, n = fix_code(masked)
    restored = re.sub(r'\x00(\d+)\x00', lambda m: keep[int(m.group(1))], fixed)
    if restored != src:
        open(path, 'w', encoding='utf-8', errors='surrogateescape').write(restored)
    return n

if __name__ == '__main__':
    for f in sys.argv[1:]:
        print(f, fix_file(f))
