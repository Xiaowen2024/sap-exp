"""Inspect actual archive coverage before selecting a robot configuration."""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path,PurePosixPath
import re
import tarfile
import time
from .io_utils import write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--task',type=int,default=50)
    a=p.parse_args();counts=Counter();episodes=set();matches=[];last=0.;start=time.monotonic()
    with tarfile.open(a.archive,'r|gz') as archive:
        for i,m in enumerate(archive):
            mt=re.search(r'task_([0-9]+)',m.name)
            if mt:
                counts[mt.group(1)]+=1
                if int(mt.group(1))==a.task:
                    matches.append(m.name)
                    for part in PurePosixPath(m.name).parts:
                        if part.startswith('task_'):episodes.add(part)
            if time.monotonic()-last>10:
                print(i,m.name,'task50_files',len(matches),flush=True);last=time.monotonic()
    report={'archive':str(a.archive),'task_counts':dict(counts),'task50_matches':matches,
            'task50_episode_folders':sorted(episodes),'elapsed_s':time.monotonic()-start}
    write_json(a.out,report);print('complete',len(episodes),'task50 folders',flush=True)


if __name__=='__main__':main()
