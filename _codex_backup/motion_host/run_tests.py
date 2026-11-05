#!/usr/bin/env python3
"""Linux/WSL software checks of the actual motion sources; never drives hardware."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile

here = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--project-root', type=Path, default=here.parents[2])
args = parser.parse_args()
cc = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not cc:
    raise SystemExit('Host C compiler gcc/clang not found')
source = args.project_root / 'app_program/subsystem2'
with tempfile.TemporaryDirectory(prefix='motion_host_') as tmp:
    for label in ('hw', 'control', 'startup'):
        target = Path(tmp) / label
        cmd = [cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
               '-Wno-int-to-pointer-cast', '-I', str(here / 'mock'),
               '-I', str(source), str(here / f'test_{label}.c'), '-o', str(target)]
        subprocess.run(cmd, check=True)
        subprocess.run([str(target)], check=True)
print('PASS: all motion feedback and startup regressions (software only)')
