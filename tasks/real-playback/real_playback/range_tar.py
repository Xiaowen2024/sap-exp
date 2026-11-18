"""Read members of a remote .tar.gz from an arbitrary byte range.

A gzip stream cannot normally be entered mid-way: the inflater needs the
previous 32 KiB of output.  We (1) search the fetched range for a deflate
block boundary by trial decoding at every bit offset, (2) decode from it twice
with two different dummy 32-KiB dictionaries, and (3) keep only output bytes
that agree between the two decodes.  A byte that depends, directly or through
a chain of back-references, on the unknown window differs between the two
decodes, so every agreeing byte is exact; after the first fully-agreeing
32-KiB window all later output is exact.  Tar headers are then located by the
`ustar` magic and verified by their header checksum.

Integrity note: a range read cannot verify the archive's SHA256 or the gzip
CRC32 (both cover the whole stream).  Extracted members are checked by
successful inflate, tar header checksums, exact member size, and downstream
parsing; this is recorded in every manifest written here.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import json
import os
from pathlib import Path
import tarfile
import time
import urllib.request
import zlib

import numpy as np

WINDOW = 32768
DICT_A = b'\x00' * WINDOW
DICT_B = b'\xa5' * WINDOW


def fetch(url, lo, hi, retries=6):
    """Inclusive byte range."""
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers={'Range': f'bytes={lo}-{hi}',
                                                       'User-Agent': 'range-tar/1'})
            with urllib.request.urlopen(req, timeout=60) as r:
                if r.status != 206:
                    raise IOError(f'status {r.status}')
                data = r.read()
            if len(data) != hi - lo + 1:
                raise IOError('short range')
            return data
        except Exception:
            if attempt == retries - 1:
                raise
            time.sleep(2 + 3 * attempt)


def fetch_parallel(url, lo, hi, chunk=4 << 20, workers=12, sink=None):
    """Fetch [lo, hi] inclusive; returns bytes, or writes to sink file at offset."""
    starts = list(range(lo, hi + 1, chunk))
    out = {}
    with cf.ThreadPoolExecutor(workers) as ex:
        futs = {ex.submit(fetch, url, s, min(hi, s + chunk - 1)): s for s in starts}
        for f in cf.as_completed(futs):
            s = futs[f]
            data = f.result()
            if sink is not None:
                sink.seek(s - lo)
                sink.write(data)
            else:
                out[s] = data
    if sink is not None:
        return None
    return b''.join(out[s] for s in starts)


def shifted(buf, s):
    """Bytes whose bit 0 is bit s of buf (deflate is LSB-first)."""
    if s == 0:
        return bytes(buf)
    a = np.frombuffer(buf, np.uint8).astype(np.uint16)
    b = np.r_[a[1:], 0]
    return ((a >> s) | (b << (8 - s))).astype(np.uint8).tobytes()


def _try(data, need):
    d = zlib.decompressobj(-15, zdict=DICT_A)
    try:
        out = d.decompress(data, need)
    except zlib.error:
        return False
    return len(out) >= need


def find_block(buf, start=0, probe=1 << 16, need=1 << 18, limit=None):
    """Return (byte_index, bit_shift) of the first plausible block start."""
    shifts = [shifted(buf, s) for s in range(8)]
    stop = len(buf) - probe if limit is None else min(len(buf) - probe, start + limit)
    raw = np.frombuffer(buf, np.uint8).astype(np.uint32)
    cands = []
    for s in range(8):
        arr = np.frombuffer(shifts[s], np.uint8)[start:stop + 1].astype(np.uint32)
        b0, b1 = arr[:-1], arr[1:]
        dyn = ((b0 & 7) == 4) & ((b0 >> 3) <= 29) & ((b1 & 31) <= 29)
        # stored block: 3 header bits (000) then pad to the next byte boundary
        # of the original stream, then LEN, NLEN.
        j = np.arange(start, stop) + (1 if s <= 5 else 2)
        ok = j + 4 < len(buf)
        jj = np.where(ok, j, 0)
        ln = raw[jj] | raw[np.minimum(jj + 1, len(raw) - 1)] << 8
        nl = raw[np.minimum(jj + 2, len(raw) - 1)] | raw[np.minimum(jj + 3, len(raw) - 1)] << 8
        sto = ((b0 & 7) == 0) & ok & ((ln ^ nl) == 0xFFFF) & (ln > 0)
        for i in np.nonzero(dyn | sto)[0]:
            cands.append((int(i) + start, s))
    cands.sort()
    for i, s in cands:
        if _try(shifts[s][i:i + probe], 1 << 12) and _try(shifts[s][i:], need):
            return i, s
    raise ValueError('no deflate block boundary found')


def decode_exact(buf, i, s):
    """Inflate from block start; return (exact_bytes, offset_in_output_where_exact_begins)."""
    data = shifted(buf, s)[i:]
    outs = []
    for zd in (DICT_A, DICT_B):
        d = zlib.decompressobj(-15, zdict=zd)
        try:
            o = d.decompress(data)
        except zlib.error:
            o = b''
        # On a truncated range the tail just stops; zlib raises only on corrupt data.
        outs.append(o)
    a, b = outs
    n = min(len(a), len(b))
    av = np.frombuffer(a[:n], np.uint8)
    bv = np.frombuffer(b[:n], np.uint8)
    diff = np.nonzero(av != bv)[0]
    # exact from the end of the last differing byte onwards (all later bytes are
    # literal-derived or copy exact bytes once a full window agrees)
    first = int(diff[-1]) + 1 if len(diff) else 0
    return a[first:n], first


def robust_block(buf, limit, min_exact=1 << 20):
    """Skip false block starts: a true start inflates to the end of the range."""
    start = 0
    for _ in range(200):
        i, s = find_block(buf, start=start, limit=limit - start)
        exact, _ = decode_exact(buf, i, s)
        if len(exact) >= min(min_exact, len(buf) // 2):
            return i, s, exact
        start = i + 1
    raise ValueError('no block start decodes the range')


def scan_headers(data):
    """Yield (offset, TarInfo) for every checksum-valid header, walking members."""
    pos = 0
    found = []
    while True:
        k = data.find(b'ustar', pos)
        if k < 0:
            return found
        h = k - 257
        if h >= 0 and h + 512 <= len(data):
            try:
                ti = tarfile.TarInfo.frombuf(data[h:h + 512], 'utf-8', 'surrogateescape')
                break
            except Exception:
                pass
        pos = k + 1
    while h + 512 <= len(data):
        block = data[h:h + 512]
        if block == b'\0' * 512:
            break
        try:
            ti = tarfile.TarInfo.frombuf(block, 'utf-8', 'surrogateescape')
        except Exception:
            break
        found.append((h, ti))
        h += 512 + ((ti.size + 511) // 512) * 512
    return found


def probe(url, offset, length=6 << 20):
    buf = fetch_parallel(url, offset, offset + length - 1, chunk=1 << 20, workers=6)
    i, s, exact = robust_block(buf, 4 << 20)
    hs = scan_headers(exact)
    names = [ti.name for _, ti in hs]
    return {'offset': offset, 'block_byte': offset + i, 'shift': s, 'exact_bytes': len(exact),
            'first': names[0] if names else None, 'last': names[-1] if names else None,
            'n_headers': len(names)}


def extract(url, lo, hi, prefix_filter, out_dir, part_file):
    """Fetch [lo,hi], inflate exactly, write members whose name contains prefix_filter."""
    out_dir = Path(out_dir)
    part_file = Path(part_file)
    if not part_file.exists() or part_file.stat().st_size != hi - lo + 1:
        with open(part_file, 'wb') as f:
            f.truncate(hi - lo + 1)
            fetch_parallel(url, lo, hi, chunk=8 << 20, workers=12, sink=f)
    buf = part_file.read_bytes()
    i, s, exact = robust_block(buf, 8 << 20)
    del buf
    hs = scan_headers(exact)
    written = []
    complete = True
    for h, ti in hs:
        if prefix_filter not in ti.name:
            continue
        if not ti.isfile():
            continue
        body = exact[h + 512:h + 512 + ti.size]
        if len(body) != ti.size:
            complete = False
            continue
        rel = Path(ti.name)
        if rel.is_absolute() or '..' in rel.parts:
            raise ValueError('unsafe member name ' + ti.name)
        dst = out_dir / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(body)
        written.append({'name': ti.name, 'size': ti.size, 'sha256': hashlib.sha256(body).hexdigest()})
    names = [ti.name for _, ti in hs]
    return {'range': [lo, hi], 'block_byte': lo + i, 'shift': s, 'members_seen': len(names),
            'first_member': names[0] if names else None, 'last_member': names[-1] if names else None,
            'written': written, 'all_matching_members_complete': complete,
            'integrity': 'range read: inflate success + tar header checksums + exact sizes; '
                         'archive SHA256/gzip CRC not verifiable on a range'}


def stream_extract(part_file, base_offset, match, out_dir, search_limit=8 << 20, feed=1 << 20):
    """Continuous exact inflate of a fetched range; writes members whose name contains `match`.

    Two inflaters (dummy dictionaries A and B) run until a 32-KiB window of
    their outputs agrees; from then on output A is exact and B is dropped.
    """
    out_dir = Path(out_dir)
    buf_head = open(part_file, 'rb').read(search_limit + (1 << 17))
    start = 0
    while True:
        i, s = find_block(buf_head, start=start, limit=search_limit - start)
        # validate: this start reaches a 32-KiB agreeing window within the head
        a, _ = decode_exact(buf_head, i, s)
        if len(a) >= WINDOW:
            break
        start = i + 1
    size = os.path.getsize(part_file)
    f = open(part_file, 'rb')
    f.seek(i)
    carry_bits = None
    da = zlib.decompressobj(-15, zdict=DICT_A)
    db = zlib.decompressobj(-15, zdict=DICT_B)
    pend_a = bytearray(); pend_b = bytearray()
    synced = False
    tail = b''  # bit-shift carry
    tarbuf = bytearray()
    state = {'aligned': False, 'need': 0, 'member': None, 'fh': None, 'skip': 0}
    written = []; seen = []
    consumed = 0
    stream_end = False

    def shift_chunk(chunk, nxt):
        if s == 0:
            return chunk
        x = np.frombuffer(chunk + nxt[:1], np.uint8).astype(np.uint16)
        return ((x[:-1] >> s) | (x[1:] << (8 - s))).astype(np.uint8).tobytes()

    def tar_feed(data):
        tarbuf.extend(data)
        pos = 0
        if not state['aligned']:
            k = bytes(tarbuf).find(b'ustar')
            while k >= 0:
                h = k - 257
                if h >= 0 and h + 512 <= len(tarbuf):
                    try:
                        tarfile.TarInfo.frombuf(bytes(tarbuf[h:h + 512]), 'utf-8', 'surrogateescape')
                        state['aligned'] = True
                        pos = h
                        break
                    except Exception:
                        pass
                elif h + 512 > len(tarbuf):
                    break
                k = bytes(tarbuf).find(b'ustar', k + 1)
            if not state['aligned']:
                keep = max(0, len(tarbuf) - 1024)
                del tarbuf[:keep]
                return
        while True:
            if state['member'] is not None:
                n = min(state['need'], len(tarbuf) - pos)
                if state['fh'] is not None and n:
                    state['fh'].write(tarbuf[pos:pos + n]); state['hash'].update(tarbuf[pos:pos + n])
                state['need'] -= n; pos += n
                if state['need']:
                    break
                if state['fh'] is not None:
                    state['fh'].close()
                    written.append({'name': state['member'].name, 'size': state['member'].size,
                                    'sha256': state['hash'].hexdigest()})
                state['member'] = None; state['fh'] = None
                state['skip'] = (-state['size']) % 512
            if state['skip']:
                n = min(state['skip'], len(tarbuf) - pos)
                state['skip'] -= n; pos += n
                if state['skip']:
                    break
            if len(tarbuf) - pos < 512:
                break
            block = bytes(tarbuf[pos:pos + 512])
            if block == b'\0' * 512:
                pos += 512
                continue
            ti = tarfile.TarInfo.frombuf(block, 'utf-8', 'surrogateescape')
            pos += 512
            seen.append(ti.name)
            state.update(member=ti, need=ti.size if ti.isfile() else 0, size=ti.size if ti.isfile() else 0, fh=None)
            if ti.isfile() and match in ti.name:
                rel = Path(ti.name)
                if rel.is_absolute() or '..' in rel.parts:
                    raise ValueError('unsafe member ' + ti.name)
                dst = out_dir / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                state['fh'] = open(dst, 'wb'); state['hash'] = hashlib.sha256()
        del tarbuf[:pos]

    pos = i
    nxt = f.read(feed)
    while nxt:
        chunk = nxt
        nxt = f.read(feed)
        data = shift_chunk(chunk, nxt if nxt else b'\0')
        try:
            oa = da.decompress(data)
            ob = db.decompress(data) if not synced else None
        except zlib.error as e:
            raise RuntimeError(f'inflate error near byte {pos}: {e}')
        pos += len(chunk)
        if da.eof:
            stream_end = True
        if not synced:
            pend_a.extend(oa); pend_b.extend(ob)
            n = min(len(pend_a), len(pend_b))
            av = np.frombuffer(bytes(pend_a[:n]), np.uint8); bv = np.frombuffer(bytes(pend_b[:n]), np.uint8)
            d = np.nonzero(av != bv)[0]
            last = int(d[-1]) + 1 if len(d) else 0
            if n - last >= WINDOW:
                synced = True
                tar_feed(bytes(pend_a[last:n]))
                if len(pend_a) > n:
                    tar_feed(bytes(pend_a[n:]))
                pend_a = pend_b = None; db = None
        else:
            tar_feed(oa)
        if stream_end:
            break
    if state['fh'] is not None:
        state['fh'].close()
        truncated = state['member'].name
        (out_dir / truncated).unlink()
    else:
        truncated = None
    return {'block_byte': base_offset + i, 'shift': s, 'synced': synced, 'members_seen': len(seen),
            'first_member': seen[0] if seen else None, 'last_member': seen[-1] if seen else None,
            'written': written, 'truncated_member_dropped': truncated,
            'integrity': 'range read: inflate success + exact-window sync + tar header checksums + exact sizes; '
                         'archive SHA256/gzip CRC not verifiable on a range'}


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('probe'); a.add_argument('url'); a.add_argument('offsets', type=int, nargs='+')
    a = sub.add_parser('extract'); a.add_argument('url'); a.add_argument('lo', type=int); a.add_argument('hi', type=int)
    a.add_argument('--match', required=True); a.add_argument('--out', required=True); a.add_argument('--part', required=True)
    a.add_argument('--report', required=True)
    args = p.parse_args()
    if args.cmd == 'probe':
        for off in args.offsets:
            t = time.time()
            try:
                r = probe(args.url, off)
            except Exception as e:
                r = {'offset': off, 'error': repr(e)}
            r['s'] = round(time.time() - t, 1)
            print(json.dumps(r), flush=True)
    else:
        part = Path(args.part)
        ok = Path(str(part) + '.ok')
        if not ok.exists():
            with open(part, 'wb') as f:
                f.truncate(args.hi - args.lo + 1)
                fetch_parallel(args.url, args.lo, args.hi, chunk=8 << 20, workers=12, sink=f)
            ok.write_text('complete')
        r = stream_extract(part, args.lo, args.match, args.out)
        r['range'] = [args.lo, args.hi]; r['url'] = args.url
        Path(args.report).write_text(json.dumps(r, indent=1))
        print(json.dumps({k: v for k, v in r.items() if k != 'written'}), len(r['written']), 'files')


if __name__ == '__main__':
    main()
