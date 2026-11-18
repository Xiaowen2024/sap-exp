"""Resumable bounded-memory archive relay into GPU-host tmpfs.

All raw bytes live on the server. Workstation RAM holds at most one bounded
HTTP range per worker. Completed ranges survive process/SSH/HTTP interruption.
The complete compressed archive must match the pinned author's Git LFS SHA256
before extraction. Only task_0050 and calibration are retained persistently.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
import json
import itertools
from pathlib import Path
import shlex
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from .io_utils import GateError, write_json
from .remote import connect

REVISION='ae13524566502155941720083c9d5921e4100966'
FILES={
    'calib':('RH20T_calib',350972580,'f9d5bcfb3968eef256f500f26a1f7ee8bb223c16bb99ca31fd8e278095d252fc'),
    'color':('RH20T_color',4754051934,'e1481bb7967c44d4b7f055da4e28ea8fbae29ac4db3b6e3904fd79977b0a5b83'),
    'lowdim':('RH20T_lowdim',12139023797,'04782fcdc602e9cd20aa83af4b37abd40c637aeec69d37b65349ccffc08a69e8'),
}


def acquire(credentials,kind,remote_root,output,workers=6):
    folder,size,sha=FILES[kind]
    url=f'https://huggingface.co/datasets/tonyfanggalaxies/RH20T/resolve/{REVISION}/{folder}/RH20T_cfg3.tar.gz'
    chunk=1024*1024;count=(size+chunk-1)//chunk
    archive=f'/dev/shm/real_playback_v1/{kind}_cfg3.tar.gz.part'
    state_path=f'{remote_root}/download_state/{kind}_cfg3.json'
    status=Path(output)/f'acquire_{kind}_cfg3.json'
    client=connect(credentials)
    _,o,e=client.exec_command('mkdir -p /dev/shm/real_playback_v1 '+shlex.quote(remote_root+'/download_state'))
    if o.channel.recv_exit_status()!=0:raise GateError(e.read().decode())
    meta=client.open_sftp()
    try:
        with meta.open(state_path) as f:state=json.load(f)
        if (state['size'],state['sha256'],state['chunk_bytes'])!=(size,sha,chunk):raise GateError('resume source fingerprint differs')
        stat=meta.stat(archive)
        if stat.st_size!=size:raise GateError('resume archive length changed')
    except IOError:
        with meta.open(archive,'wb') as f:f.truncate(size)
        state={'kind':kind,'size':size,'sha256':sha,'chunk_bytes':chunk,'completed':[],
               'source_url':url,'revision':REVISION,'archive_on_gpu_host':archive}
    done=set(state['completed']);lock=threading.Lock();local=threading.local();signed={'url':None,'time':0.}
    started=time.monotonic();initial=len(done);errors={}
    def save(stage):
        state['completed']=sorted(done);state['status']=stage;state['updated_unix']=time.time()
        with meta.open(state_path+'.tmp','w') as f:f.write(json.dumps(state))
        meta.rename(state_path+'.tmp',state_path) if not _exists(meta,state_path) else meta.posix_rename(state_path+'.tmp',state_path)
        write_json(status,{'kind':kind,'status':stage,'total_bytes':size,
                           'complete_ranges':len(done),'total_ranges':count,
                           'bytes_confirmed':sum(min(chunk,size-i*chunk) for i in done),
                           'archive_sha256_expected':sha,'raw_local_files':False,
                           'updated_unix':time.time(),'recent_errors':errors})
    def direct_url(refresh=False):
        with lock:
            if refresh or signed['url'] is None or time.monotonic()-signed['time']>1200:
                request_url=url+'?download=true&relay_epoch='+str(int(time.time()//600))
                with urlopen(request_url,timeout=30) as r:
                    if int(r.headers.get('Content-Length',0))!=size:raise GateError('author archive length changed')
                    signed.update(url=r.geturl(),time=time.monotonic())
            return signed['url']
    def transfer(i):
        if not hasattr(local,'sftp'):local.sftp=client.open_sftp()
        lo=i*chunk;hi=min(size,lo+chunk)-1
        req=Request(direct_url(),headers={'Range':f'bytes={lo}-{hi}'})
        with urlopen(req,timeout=30) as response:
            if response.status!=206 or response.headers.get('Content-Range')!=f'bytes {lo}-{hi}/{size}':
                raise GateError('range response header mismatch')
            data=response.read(hi-lo+2)
            if len(data)!=hi-lo+1:raise GateError('truncated HTTP range')
        with local.sftp.open(archive,'r+b',bufsize=0) as file:
            file.set_pipelined(True);file.seek(lo);file.write(data)
        return i
    try:
        save('downloading verified ranges')
        while len(done)<count:
            pending=[i for i in range(count) if i not in done]
            with ThreadPoolExecutor(workers) as pool:
                iterator=iter(pending)
                futures={pool.submit(transfer,i):i for i in itertools.islice(iterator,workers)}
                last_save=time.monotonic()
                while futures:
                    completed,_=wait(futures,return_when=FIRST_COMPLETED)
                    for future in completed:
                        i=futures.pop(future)
                        try:done.add(future.result());errors.pop(str(i),None)
                        except Exception as exc:
                            errors[str(i)]=type(exc).__name__
                            if isinstance(exc,HTTPError) and exc.code==429:
                                time.sleep(min(600,int(exc.headers.get('Retry-After','60'))))
                        nxt=next(iterator,None)
                        if nxt is not None:futures[pool.submit(transfer,nxt)]=nxt
                    if time.monotonic()-last_save>=10:
                        save('downloading verified ranges');last_save=time.monotonic()
                        print(kind,len(done),'/',count,'ranges',flush=True)
            if len(done)<count:
                save('retrying incomplete ranges');signed['url']=None;time.sleep(10)
        save('verifying archive on GPU host')
        command='sha256sum '+shlex.quote(archive)
        _,o,e=client.exec_command(command,timeout=180)
        actual=o.read().decode().split()[0]
        if actual!=sha:raise GateError('archive SHA256 differs from pinned author release')
        state['archive_sha256_verified']=True;save('archive verified; extracting selected task')
        script=f'''from pathlib import Path
from scripts.real_playback.download import fetch_task_archive
with Path({archive!r}).open('rb') as stream:
 fetch_task_archive({url!r},Path({remote_root!r})/'data',kind={kind+'_cfg3'!r},revision={REVISION!r},max_extract_bytes=1800000000,stream_source=stream)
'''
        stdin,o,e=client.exec_command('cd '+shlex.quote(remote_root)+' && /root/autodl-tmp/venvs/mjwarp400-cu12/bin/python - >'+shlex.quote(remote_root+'/'+kind+'_extract.log')+' 2>&1')
        stdin.write(script);stdin.channel.shutdown_write();rc=o.channel.recv_exit_status()
        if rc:raise GateError(f'remote extraction failed; see {kind}_extract.log')
        save('complete; author archive SHA256 verified and task extracted')
    finally:
        meta.close();client.close()
    return state


def _exists(sftp,path):
    try:sftp.stat(path);return True
    except IOError:return False


def main():
    p=argparse.ArgumentParser();p.add_argument('--credentials',type=Path,required=True)
    p.add_argument('--kind',choices=list(FILES),required=True);p.add_argument('--remote-root',default='/root/real_playback_v1')
    p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=6)
    a=p.parse_args()
    import paramiko
    while True:
        try:
            acquire(a.credentials,a.kind,a.remote_root,a.out,a.workers)
            break
        except (OSError,EOFError,paramiko.SSHException) as exc:
            print('transport interrupted; resuming confirmed ranges',type(exc).__name__,flush=True)
            time.sleep(10)


if __name__=='__main__':main()
