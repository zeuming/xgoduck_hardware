#!/usr/bin/env python3
"""Prepare an OFFLINE candidate from the exact historical installed source.

This tool never accesses robot devices, SSH, or upload tools. It refuses an
existing output directory and a base whose source/application hashes differ.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    repo=here.parents[2]
    spec=json.loads((here/'base.json').read_text())
    base=repo/spec['base']
    def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    assert sha(base/'.pio/build/esp32s3_motion/firmware.bin')==spec['historically_installed_app_sha256']
    for name,digest in spec['source_files'].items():assert sha(base/name)==digest,name
    if args.output.exists():raise ValueError('Output already exists; refusing overwrite')
    shutil.copytree(base,args.output,ignore=shutil.ignore_patterns('.pio','__pycache__'))
    subprocess.run(['patch','--batch','-p1','-i',str(here/'candidate.patch')],cwd=args.output,check=True)
    shutil.copyfile(here/'test_position_retry.cpp',args.output/'tests/test_position_retry.cpp')
    run=args.output/'tests/run.sh'
    script=run.read_text().replace('/Users/lo/Documents/project/xgoduck_hardware',str(repo))
    script+='\nc++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \\\n  -I "$task_dir/tests/mocks" "$task_dir/tests/test_position_retry.cpp" -o "$task_build/position-retry"\n"$task_build/position-retry"\n'
    run.write_text(script)
    print('Prepared offline candidate; not built, uploaded, or physically validated.')


if __name__=='__main__':main()
