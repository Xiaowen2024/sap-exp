"""Restricted RH20T NumPy loading, content hashes, and atomic artifacts.

Official transformed RH20T files contain dictionaries in object NPY files.
They cannot be read with ordinary ``np.load`` without enabling arbitrary
pickle globals. Here only NumPy's numeric reconstruction globals are allowed;
the result is recursively checked before use. Dataset modules are never run.
"""
from __future__ import annotations

import hashlib
import io
import json
import math
import pickle
from pathlib import Path

import numpy as np


class GateError(ValueError):
    """A required protocol gate has not passed."""


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def read_json(path):
    def reject(x):
        raise GateError(f'non-finite JSON number: {x}')
    return json.loads(Path(path).read_text(), parse_constant=reject)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


class NumericUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        # These are the NumPy 1.x and 2.x spellings used by official arrays.
        if module in ('numpy.core.multiarray', 'numpy._core.multiarray'):
            if name in ('_reconstruct', 'scalar'):
                return getattr(np._core.multiarray, name)
        if module == 'numpy' and name in ('ndarray', 'dtype'):
            return getattr(np, name)
        raise GateError(f'prohibited pickle global: {module}.{name}')

    def persistent_load(self, pid):
        raise GateError('persistent pickle references are prohibited')


def _check_tree(x, seen=None, depth=0):
    if depth > 80:
        raise GateError('nested data exceeds limit')
    seen = set() if seen is None else seen
    if isinstance(x, (dict, list, tuple, np.ndarray)):
        if id(x) in seen:
            raise GateError('cyclic or aliased object tree')
        seen.add(id(x))
    if type(x) in (str, bytes, bool, int, float, type(None)):
        if isinstance(x, float) and not math.isfinite(x):
            raise GateError('non-finite dataset scalar')
    elif isinstance(x, np.ndarray):
        if x.dtype.hasobject:
            for item in x.flat:
                _check_tree(item, seen, depth + 1)
        elif x.dtype.kind not in 'biufcSU':
            raise GateError(f'unsupported dtype: {x.dtype}')
        elif x.dtype.kind in 'fc' and not np.isfinite(x).all():
            raise GateError('non-finite dataset array')
    elif isinstance(x, np.generic):
        _check_tree(x.item(), seen, depth + 1)
    elif type(x) is dict:
        for key, val in x.items():
            _check_tree(key, seen, depth + 1)
            _check_tree(val, seen, depth + 1)
    elif type(x) in (list, tuple):
        for val in x:
            _check_tree(val, seen, depth + 1)
    else:
        raise GateError(f'unsupported dataset type: {type(x).__name__}')
    if isinstance(x, (dict, list, tuple, np.ndarray)):
        seen.remove(id(x))


def restricted_npy(path, max_bytes=512 * 1024 * 1024):
    path = Path(path)
    if path.stat().st_size > max_bytes:
        raise GateError(f'file exceeds restricted-loader limit: {path.name}')
    with path.open('rb') as stream:
        version = np.lib.format.read_magic(stream)
        if version == (1, 0):
            shape, fortran, dtype = np.lib.format.read_array_header_1_0(stream)
        elif version == (2, 0):
            shape, fortran, dtype = np.lib.format.read_array_header_2_0(stream)
        else:
            raise GateError(f'unsupported NPY version: {version}')
        if math.prod(shape) * max(1, dtype.itemsize) > max_bytes:
            raise GateError('NPY declared allocation exceeds limit')
        if dtype.hasobject:
            value = NumericUnpickler(io.BytesIO(stream.read())).load()
            if not isinstance(value, np.ndarray) or value.shape != shape or value.dtype != dtype:
                raise GateError('NPY header/payload mismatch')
        else:
            stream.seek(0)
            value = np.load(stream, allow_pickle=False)
    _check_tree(value)
    return value.item() if value.shape == () and value.dtype.hasobject else value


def verified_file(root, spec):
    """Resolve only a frozen, hashed input underneath its declared root."""
    root = Path(root).resolve()
    path = (root / spec['path']).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise GateError(f'input missing or escapes root: {spec["path"]}')
    if sha256(path) != spec['sha256']:
        raise GateError(f'input hash changed: {spec["path"]}')
    return path
