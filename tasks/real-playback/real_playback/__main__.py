"""Command line entry points; no implicit downloads or simulation on import."""
from __future__ import annotations

import argparse
from pathlib import Path

from .io_utils import GateError, read_json, write_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('audit');a.add_argument('--root',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('admission');a.add_argument('--episode',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('validate');a.add_argument('--manifest',type=Path,required=True);a.add_argument('--input-root',type=Path)
    a=sub.add_parser('declare');a.add_argument('--manifest',type=Path,required=True);a.add_argument('--input-root',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('plan');a.add_argument('--manifest',type=Path,required=True);a.add_argument('--split',choices=['selection','held_out'],required=True);a.add_argument('--budget',type=int,choices=[1,5],required=True);a.add_argument('--winners',type=Path);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('execute');a.add_argument('--manifest',type=Path,required=True);a.add_argument('--job',type=Path,required=True);a.add_argument('--input-root',type=Path,required=True);a.add_argument('--evidence',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('api-probe');a.add_argument('--engine',choices=['sap','mujoco','ipc'],required=True);a.add_argument('--out',type=Path,required=True)
    a=sub.add_parser('remote-probe');a.add_argument('--credentials',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    if args.command=='audit':
        from .audit import inventory
        r=inventory(args.root,args.out);print(r['status'],r['episodes_found'])
    elif args.command=='admission':
        from .completion import admission
        r=admission(read_json(args.episode));write_json(args.out,r);print('admitted',r['admitted'])
    elif args.command=='validate':
        from .manifest import validate
        validate(read_json(args.manifest),args.input_root);print('manifest valid')
    elif args.command=='declare':
        from .manifest import declare
        r=declare(read_json(args.manifest),args.out,args.input_root);print(r['declaration_sha256'])
    elif args.command=='plan':
        from .manifest import validate
        from .matrix import jobs
        m=read_json(args.manifest);validate(m)
        if args.split=='held_out' and args.budget==5 and not args.winners:raise GateError('P4 winners required')
        r=jobs(m,args.split,args.budget,read_json(args.winners) if args.winners else None)
        write_json(args.out,r);print(len(r),'jobs',sum(j['R'] for j in r),'rollouts')
    elif args.command=='execute':
        from .runner import execute_job
        r=execute_job(read_json(args.manifest),read_json(args.job),args.input_root,args.out,read_json(args.evidence));print('complete',r['complete'])
    elif args.command=='api-probe':
        from .preflight import api_probe
        r=api_probe(args.engine,args.out);print('API probe passed',r['passed'])
        if not r['passed']:raise SystemExit(2)
    elif args.command=='remote-probe':
        from .remote import probe
        r=probe(args.credentials,args.out);print('connected',r['connected'])


if __name__=='__main__':
    main()
