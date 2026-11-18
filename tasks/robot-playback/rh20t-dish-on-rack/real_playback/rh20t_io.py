"""Restricted loading of RH20T `transformed/*.npy` files (pickled dicts).

RH20T stores each stream with ``np.save(dict)``. ``np.load(allow_pickle=True)``
would run arbitrary pickle opcodes from a downloaded file, so only the numpy /
builtin classes needed to rebuild arrays, dicts and lists are admitted.
"""
from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
from numpy.lib import format as npformat

_ALLOWED = {
    ("numpy.core.multiarray", "_reconstruct"), ("numpy._core.multiarray", "_reconstruct"),
    ("numpy", "ndarray"), ("numpy", "dtype"),
    ("numpy.core.multiarray", "scalar"), ("numpy._core.multiarray", "scalar"),
    ("builtins", "dict"), ("builtins", "list"), ("builtins", "tuple"), ("builtins", "str"),
    ("builtins", "int"), ("builtins", "float"), ("builtins", "bool"), ("collections", "OrderedDict"),
    ("codecs", "encode"),
}


class _SafeUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if (module, name) in _ALLOWED:
            return super().find_class(module, name)
        raise pickle.UnpicklingError(f"blocked class {module}.{name}")


def load_npy(path):
    with open(path, "rb") as f:
        version = npformat.read_magic(f)
        if version == (1, 0):
            shape, fortran, dtype = npformat.read_array_header_1_0(f)
        else:
            shape, fortran, dtype = npformat.read_array_header_2_0(f)
        if not dtype.hasobject:
            return np.fromfile(f, dtype=dtype).reshape(shape, order="F" if fortran else "C")
        obj = _SafeUnpickler(f, encoding="latin1").load()
    if isinstance(obj, np.ndarray) and obj.dtype == object and obj.shape == ():
        obj = obj.item()
    return obj


def load_json(path):
    import json
    return json.loads(Path(path).read_text())
