"""Bounded, streaming archive acquisition on the GPU server only.

No archive is stored. Only selected regular files are extracted, never links
or device nodes. A pinned author-repository revision and each retained file's
SHA256 are recorded. Partial archive traversal never claims full-archive hash
verification. The HTTP mirror is a transport for the pinned author release.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import time
import sys
from urllib.request import Request, urlopen

from .io_utils import GateError, read_json, write_json


class ProgressReader(io.RawIOBase):
    def __init__(self, response, callback):
        self.response, self.callback, self.total = response, callback, 0
        self.last = 0
        self.sha256 = hashlib.sha256()

    def readable(self):
        return True

    def read(self, n=-1):
        data = self.response.read(n if n >= 0 else 1024*1024)
        self.total += len(data)
        self.sha256.update(data)
        if time.monotonic()-self.last >= 10:
            self.callback(self.total)
            self.last = time.monotonic()
        return data


def fetch_task_archive(url, output, *, kind, revision, max_extract_bytes=2_000_000_000,
                       metadata_only=False, stop_after_task=False, stream_source=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    status = output/f'download_{kind}.json'
    report = {'url': url, 'source_revision': revision, 'kind': kind,
              'status': 'downloading', 'compressed_bytes_read': 0,
              'retained_bytes': 0, 'files': [], 'full_archive_sha256_verified': False,
              'partial_traversal': True, 'started_unix': time.time()}
    def progress(n):
        report['compressed_bytes_read'] = n
        report['updated_unix'] = time.time()
        write_json(status, report)
        print(kind, n, 'compressed bytes;', len(report['files']), 'retained files', flush=True)
    seen_task = False
    try:
        req = Request(url, headers={'User-Agent': 'SIM-EXP-real-playback/1'})
        connection = urlopen(req, timeout=60) if stream_source is None else contextlib.nullcontext(stream_source)
        with connection as response:
            if stream_source is None and 'text/html' in response.headers.get('Content-Type', ''):
                raise GateError('download returned HTML, not an archive')
            reader = ProgressReader(response, progress)
            with tarfile.open(fileobj=reader, mode='r|gz') as archive:
                for member in archive:
                    path = PurePosixPath(member.name)
                    if path.is_absolute() or '..' in path.parts:
                        raise GateError('archive path traversal')
                    is_task = any(x.startswith('task_0050_') and not x.endswith('_human') for x in path.parts)
                    if seen_task and stop_after_task and any(x.startswith('task_0051_') for x in path.parts):
                        report['status'] = 'task subset extracted; archive tail not traversed'
                        break
                    seen_task |= is_task
                    selected = member.isfile() and (kind.startswith('calib') or is_task)
                    if metadata_only:
                        selected &= path.name == 'metadata.json'
                    if not selected:
                        continue
                    # Do not extract audio/human data, or unrelated source programs.
                    if path.suffix not in ('.npy', '.npz', '.json', '.mp4', '.png', '.jpg'):
                        continue
                    if member.size < 0 or report['retained_bytes']+member.size > max_extract_bytes:
                        raise GateError('selected extraction exceeds declared disk budget')
                    if shutil.disk_usage(output).free < member.size + 128*1024*1024:
                        raise GateError('not enough free space; retained files preserved')
                    target = (output/str(path)).resolve()
                    if not target.is_relative_to(output):
                        raise GateError('archive target escapes output directory')
                    target.parent.mkdir(parents=True, exist_ok=True)
                    source = archive.extractfile(member)
                    h = hashlib.sha256()
                    count = 0
                    temporary = target.with_suffix(target.suffix+'.part')
                    with temporary.open('wb') as stream:
                        while True:
                            block = source.read(1024*1024)
                            if not block:
                                break
                            count += len(block)
                            stream.write(block)
                            h.update(block)
                    if count != member.size:
                        raise GateError('truncated retained archive member')
                    temporary.replace(target)
                    report['retained_bytes'] += count
                    report['files'].append({'path': str(target.relative_to(output)), 'bytes': count,
                                            'sha256': h.hexdigest()})
                else:
                    report['partial_traversal'] = False
                    report['status'] = 'archive traversed; selected subset extracted'
            if not report['partial_traversal']:
                while reader.read(1024*1024):
                    pass
                report['archive_sha256'] = reader.sha256.hexdigest()
            progress(reader.total)
        report['finished_unix'] = time.time()
    except Exception as exc:
        report.update(status='download failed', error_type=type(exc).__name__, error=str(exc))
        write_json(status, report)
        raise
    write_json(status, report)
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--url', required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--kind', required=True)
    p.add_argument('--revision', required=True)
    p.add_argument('--max-extract-bytes', type=int, default=2_000_000_000)
    p.add_argument('--metadata-only', action='store_true')
    p.add_argument('--stop-after-task', action='store_true')
    p.add_argument('--stdin', action='store_true', help='receive an SSH-relayed stream; do not fetch URL on host')
    a = p.parse_args()
    fetch_task_archive(a.url, a.out, kind=a.kind, revision=a.revision,
                       max_extract_bytes=a.max_extract_bytes, metadata_only=a.metadata_only,
                       stop_after_task=a.stop_after_task, stream_source=sys.stdin.buffer if a.stdin else None)


if __name__ == '__main__':
    main()
