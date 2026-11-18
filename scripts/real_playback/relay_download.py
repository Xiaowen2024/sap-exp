"""Relay author archive bytes over SSH without storing raw data locally.

The remote downloader streams tar members directly to its isolated data root.
This is useful where a GPU host cannot reach the primary download endpoint.
Only progress and source hashes are saved on the workstation.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import shlex
import time
from urllib.request import Request, urlopen

from .io_utils import write_json
from .remote import connect

REVISION = 'ae13524566502155941720083c9d5921e4100966'


def ranged_blocks(url, chunk_bytes=4*1024*1024, workers=8):
    """Bounded in-memory HTTP ranges, yielded in archive order."""
    with urlopen(url,timeout=60) as response:
        total=int(response.headers['Content-Length'])
        direct=response.geturl()
    def get(index):
        lo=index*chunk_bytes;hi=min(total,lo+chunk_bytes)-1
        req=Request(direct,headers={'Range':f'bytes={lo}-{hi}'})
        for attempt in range(4):
            try:
                with urlopen(req,timeout=90) as r:
                    if r.status!=206 or r.headers.get('Content-Range')!=f'bytes {lo}-{hi}/{total}':
                        raise RuntimeError('server did not honor bounded HTTP range')
                    data=r.read(hi-lo+2)
                    if len(data)!=hi-lo+1:raise RuntimeError('truncated HTTP range')
                    return data
            except Exception:
                if attempt==3:raise
                time.sleep(2**attempt)
    count=(total+chunk_bytes-1)//chunk_bytes
    with ThreadPoolExecutor(workers) as pool:
        pending={i:pool.submit(get,i) for i in range(min(workers,count))}
        for i in range(count):
            data=pending.pop(i).result()
            if i+workers<count:pending[i+workers]=pool.submit(get,i+workers)
            yield data


def relay(credentials, config, kind, remote_root, output):
    folder = {'calib': 'RH20T_calib', 'color': 'RH20T_color', 'lowdim': 'RH20T_lowdim'}[kind]
    url = f'https://huggingface.co/datasets/tonyfanggalaxies/RH20T/resolve/{REVISION}/{folder}/RH20T_cfg{config}.tar.gz?download=true'
    report = {'kind': kind, 'config': config, 'source_url': url, 'revision': REVISION,
              'raw_local_files': False, 'bytes_relayed': 0, 'status': 'connecting', 'started_unix': time.time()}
    status = Path(output)/f'relay_{kind}_cfg{config}.json'
    write_json(status, report)
    c = connect(credentials)
    argv = ['/root/autodl-tmp/venvs/mjwarp400-cu12/bin/python', '-u', '-m', 'scripts.real_playback.download',
            '--stdin', '--url', url, '--out', remote_root+'/data', '--kind', kind+f'_cfg{config}',
            '--revision', REVISION, '--max-extract-bytes', '1200000000']
    log = remote_root+f'/relay_{kind}_cfg{config}.log'
    command = 'cd '+shlex.quote(remote_root)+' && '+shlex.join(argv)+' >'+shlex.quote(log)+' 2>&1'
    stdin, stdout, stderr = c.exec_command(command, timeout=None)
    start = last = time.monotonic()
    try:
        report['status'] = 'relaying bounded parallel ranges'
        for block in ranged_blocks(url):
            stdin.write(block)
            report['bytes_relayed'] += len(block)
            if time.monotonic()-last >= 10:
                report['updated_unix'] = time.time()
                report['mean_MiB_s'] = report['bytes_relayed']/2**20/(time.monotonic()-start)
                write_json(status,report)
                print(kind,config,report['bytes_relayed'],flush=True)
                last = time.monotonic()
        stdin.channel.shutdown_write()
        rc = stdout.channel.recv_exit_status()
        report['exit_code'] = rc
        report['status'] = 'remote extraction finished' if rc == 0 else 'remote extraction failed'
    except Exception as exc:
        report.update(status='relay interrupted',error_type=type(exc).__name__,error=str(exc))
        raise
    finally:
        report['finished_unix'] = time.time()
        write_json(status,report)
        c.close()
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--credentials',type=Path,required=True)
    p.add_argument('--config',type=int,default=3)
    p.add_argument('--remote-root',default='/root/real_playback_v1')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    with ThreadPoolExecutor(3) as pool:
        futures=[pool.submit(relay,a.credentials,a.config,k,a.remote_root,a.out) for k in ('calib','color','lowdim')]
        for f in as_completed(futures):
            print(f.result()['status'],flush=True)


if __name__ == '__main__':
    main()
