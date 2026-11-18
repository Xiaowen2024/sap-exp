"""Explicit GPU-server transport. Credentials never enter artifacts."""
from __future__ import annotations

from pathlib import Path
import io
import shlex
import tarfile
import time

from .io_utils import write_json


def connect(credentials):
    import paramiko
    lines = [x.strip() for x in Path(credentials).read_text().splitlines() if x.strip()]
    args = shlex.split(lines[0])
    user, host = args[-1].split('@')
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, port=int(args[args.index('-p')+1]), username=user,
                   password=lines[1], timeout=15, auth_timeout=15,
                   look_for_keys=False, allow_agent=False)
    client.get_transport().set_keepalive(20)
    return client


def probe(credentials, output):
    report = {'kind': 'environment_readiness_not_benchmark', 'checked_unix': time.time()}
    try:
        client = connect(credentials)
        cmd = ('nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv; '
               'df -h / /root/autodl-tmp /dev/shm; '
               'find /root/autodl-tmp /root/work /root/autodl-nas '
               '-maxdepth 6 -iname "*rh20t*" 2>/dev/null | head -30')
        _, stdout, stderr = client.exec_command(cmd, timeout=45)
        report.update(connected=True, stdout=stdout.read().decode(), stderr=stderr.read().decode(),
                      exit_code=stdout.channel.recv_exit_status())
        client.close()
    except Exception as exc:
        report.update(connected=False, error_type=type(exc).__name__)
    write_json(output, report)
    return report


def stage_sources(credentials, remote_root):
    """Upload only this public package. Never sync a dirty workspace wholesale."""
    source = Path(__file__).resolve().parent
    if not remote_root.startswith('/') or any(x in remote_root for x in ('..', '\n')):
        raise ValueError('remote_root must be a dedicated absolute directory')
    client = connect(credentials)
    payload=io.BytesIO()
    with tarfile.open(fileobj=payload,mode='w:gz') as archive:
        for file in sorted(source.glob('*.py')):
            archive.add(file,arcname='scripts/real_playback/'+file.name)
    cmd='mkdir -p '+shlex.quote(remote_root)+' && tar -xzf - -C '+shlex.quote(remote_root)
    stdin,stdout,stderr=client.exec_command(cmd,timeout=45)
    stdin.write(payload.getvalue());stdin.channel.shutdown_write()
    stdout.read()
    if stdout.channel.recv_exit_status():raise RuntimeError(stderr.read().decode())
    client.close()
