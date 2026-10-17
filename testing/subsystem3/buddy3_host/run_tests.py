#!/usr/bin/env python3
"""Run Buddy3 portable and adapter host tests against the imported source."""
from pathlib import Path
import subprocess
import shutil
import tempfile
import os

here = Path(__file__).resolve().parent
project = here.parents[2]
source = project / 'app_program/subsystem3'
cc = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not cc:
    raise SystemExit('Host C compiler gcc/clang not found')
base = [cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-I', str(source), '-I', str(here)]
with tempfile.TemporaryDirectory(prefix='buddy3_host_') as td:
    for label, names, extra in [
        ('core', ['b3_code39.c', 'b3_barcode.c', 'b3_line.c'], ['test_core.c', 'test_noop_port.c']),
        ('adapters', ['b3_code39.c', 'b3_barcode.c', 'b3_line.c', 'b3_ir.c', 'buddy3.c'], ['test_adapters.c']),
    ]:
        cmd = base + ['-I', str(here/'mock')]
        cmd += [str(source/n) for n in names]
        cmd += [str(here/n) for n in extra]
        cmd += ['-o', str(Path(td)/label)]
        subprocess.run(cmd, check=True)
        subprocess.run([str(Path(td)/label)], check=True)
    subprocess.run(base + ['-I', str(here/'mock'), '-c',
                    str(source/'b3_port_mtk3_rp2040.c'), '-o',
                    str(Path(td)/'port.o')], check=True)
    print('PASS: Buddy3 port adapter mock compile-check')
