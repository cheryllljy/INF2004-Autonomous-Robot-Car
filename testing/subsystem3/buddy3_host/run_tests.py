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
    adapters = ['b3_code39.c', 'b3_barcode.c', 'b3_line.c', 'b3_ir.c', 'buddy3.c']
    # IR_B both ways: AO + adaptive threshold (B3_BARCODE_ANALOG 1) and
    # DO + GPIO edge interrupt (0), whatever b3_config.h currently selects.
    for label, names, extra, defs in [
        ('core', ['b3_code39.c', 'b3_barcode.c', 'b3_line.c'], ['test_core.c', 'test_noop_port.c'], []),
        ('adapters-analog', adapters, ['test_adapters.c'], ['-DB3_BARCODE_ANALOG=1U']),
        ('adapters-do', adapters, ['test_adapters.c'], ['-DB3_BARCODE_ANALOG=0U']),
    ]:
        cmd = base + defs + ['-I', str(here/'mock')]
        cmd += [str(source/n) for n in names]
        cmd += [str(here/n) for n in extra]
        cmd += ['-o', str(Path(td)/label)]
        subprocess.run(cmd, check=True)
        print(f'--- {label}')
        subprocess.run([str(Path(td)/label)], check=True)
    for mode in ('0U', '1U'):
        subprocess.run(base + ['-DB3_BARCODE_ANALOG=' + mode, '-I', str(here/'mock'), '-c',
                        str(source/'b3_port_mtk3_rp2040.c'), '-o',
                        str(Path(td)/'port.o')], check=True)
    print('PASS: Buddy3 port adapter mock compile-check (DO and analog)')
