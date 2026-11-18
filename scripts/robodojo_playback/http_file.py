"""Read bounded, validated HTTP ranges without downloading camera datasets."""
from collections import OrderedDict
import io
import re
import urllib.request


class HTTPFile(io.RawIOBase):
    def __init__(self, url, size, block_size=262144, cache_blocks=24):
        self.url, self.size = url, int(size)
        self.block_size, self.cache_blocks = block_size, cache_blocks
        self.position, self.transferred = 0, 0
        self.cache = OrderedDict()

    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.position

    def seek(self, offset, whence=0):
        p = offset if whence == 0 else self.position + offset if whence == 1 else self.size + offset
        if p < 0: raise ValueError('negative seek')
        self.position = p
        return p

    def _block(self, index):
        if index in self.cache:
            self.cache.move_to_end(index)
            return self.cache[index]
        start = index * self.block_size
        end = min(start + self.block_size, self.size) - 1
        # Distinct query keys prevent CDNs serving an earlier cached range.
        url = self.url + ('&' if '?' in self.url else '?') + f'range={start}-{end}'
        req = urllib.request.Request(url, headers={'Range': f'bytes={start}-{end}', 'Accept-Encoding': 'identity'})
        with urllib.request.urlopen(req, timeout=60) as response:
            expected = f'bytes {start}-{end}/{self.size}'
            if response.status != 206 or response.headers.get('Content-Range') != expected:
                raise IOError(f'HTTP range mismatch: {response.status} {response.headers.get("Content-Range")}, expected {expected}')
            data = response.read(end - start + 2)
        if len(data) != end - start + 1: raise IOError('truncated HTTP range')
        self.transferred += len(data)
        self.cache[index] = data
        if len(self.cache) > self.cache_blocks: self.cache.popitem(last=False)
        return data

    def read(self, size=-1):
        size = min(self.size - self.position, size if size >= 0 else self.size)
        chunks = []
        while size > 0:
            index, offset = divmod(self.position, self.block_size)
            data = self._block(index)
            take = min(size, len(data) - offset)
            chunks.append(data[offset:offset + take])
            self.position += take
            size -= take
        return b''.join(chunks)

    def readinto(self, buffer):
        data = self.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)
