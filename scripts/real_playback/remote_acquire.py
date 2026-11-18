"""Durable, resumable range acquisition entirely on the GPU server."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor,wait,FIRST_COMPLETED
import hashlib
import itertools
import json
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request,urlopen

from .acquire import FILES,REVISION
from .io_utils import GateError,read_json,sha256,write_json
from .download import fetch_task_archive


def acquire_server(kind,root,workers):
    root=Path(root);folder,size,sha=FILES[kind];chunk=1024*1024;count=(size+chunk-1)//chunk
    archive=Path('/dev/shm/real_playback_v1')/(kind+'_cfg3.tar.gz.part');archive.parent.mkdir(exist_ok=True)
    path=root/'download_state'/(kind+'_cfg3.json')
    if path.exists() and archive.exists():
        state=read_json(path)
        if (state['size'],state['sha256'],state['chunk_bytes'])!=(size,sha,chunk):raise GateError('resume source differs')
    else:
        with archive.open('wb') as f:f.truncate(size)
        state={'kind':kind,'size':size,'sha256':sha,'chunk_bytes':chunk,'completed':[],
               'revision':REVISION,'archive_on_gpu_host':str(archive)}
    done=set(state['completed']);lock=threading.Lock();signed={'url':None,'time':0.};errors={}
    source=f'https://huggingface.co/datasets/tonyfanggalaxies/RH20T/resolve/{REVISION}/{folder}/RH20T_cfg3.tar.gz'
    transport=source.replace('huggingface.co','hf-mirror.com')
    def resolve():
        with lock:
            if signed['url'] is None or time.monotonic()-signed['time']>1200:
                with urlopen(transport+'?download=true&relay_epoch='+str(int(time.time()//600)),timeout=30) as r:
                    if int(r.headers.get('Content-Length',0))!=size:raise GateError('source length changed')
                    signed.update(url=r.geturl(),time=time.monotonic())
            return signed['url']
    def transfer(i):
        lo=i*chunk;hi=min(size,lo+chunk)-1
        with urlopen(Request(resolve(),headers={'Range':f'bytes={lo}-{hi}'}),timeout=45) as r:
            if r.status!=206 or r.headers.get('Content-Range')!=f'bytes {lo}-{hi}/{size}':raise GateError('range mismatch')
            data=r.read(hi-lo+2)
            if len(data)!=hi-lo+1:raise GateError('truncated range')
        with archive.open('r+b',buffering=0) as f:f.seek(lo);f.write(data)
        return i
    def save(status):
        state.update(completed=sorted(done),status=status,updated_unix=time.time(),source_url=source,
                     transport_url=transport,recent_error_counts={x:list(errors.values()).count(x) for x in set(errors.values())})
        write_json(path,state)
        print(kind,len(done),'/',count,status,flush=True)
    save('server range download')
    while len(done)<count:
        remaining=iter(i for i in range(count) if i not in done)
        with ThreadPoolExecutor(workers) as pool:
            futures={pool.submit(transfer,i):i for i in itertools.islice(remaining,workers)};last=time.monotonic()
            while futures:
                ready,_=wait(futures,timeout=10,return_when=FIRST_COMPLETED)
                for f in ready:
                    i=futures.pop(f)
                    try:done.add(f.result());errors.pop(i,None)
                    except Exception as exc:
                        errors[i]=type(exc).__name__
                        if isinstance(exc,HTTPError) and exc.code==429:time.sleep(min(600,int(exc.headers.get('Retry-After','60'))))
                    nxt=next(remaining,None)
                    if nxt is not None:futures[pool.submit(transfer,nxt)]=nxt
                if time.monotonic()-last>10:save('server range download');last=time.monotonic()
        if len(done)<count:signed['url']=None;save('retrying incomplete ranges');time.sleep(10)
    save('verifying author archive SHA256')
    if sha256(archive)!=sha:raise GateError('author archive SHA256 mismatch')
    state['archive_sha256_verified']=True;save('extracting task subset')
    with archive.open('rb') as stream:
        fetch_task_archive(source,root/'data',kind=kind+'_cfg3',revision=REVISION,
                           max_extract_bytes=1800000000,stream_source=stream)
    save('complete; author SHA256 verified; task subset extracted')


def main():
    p=argparse.ArgumentParser();p.add_argument('--kind',choices=list(FILES),required=True)
    p.add_argument('--root',type=Path,default=Path('/root/real_playback_v1'));p.add_argument('--workers',type=int,default=16)
    a=p.parse_args();acquire_server(a.kind,a.root,a.workers)


if __name__=='__main__':main()
