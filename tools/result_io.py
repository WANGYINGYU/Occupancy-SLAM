"""Readers for the numeric files exported by the C++ backends (no ROS/GUI)."""
from pathlib import Path
import numpy as np


def read_pcd(path):
    """Read backend ASCII/binary PCD, preserving field offsets and intensity."""
    with Path(path).open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f'Truncated PCD header: {path}')
            line = line.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *parts = line.split()
            header[key] = parts
            if key == 'DATA':
                break
        names = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        kinds = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        if not len(names) == len(sizes) == len(kinds) == len(counts):
            raise ValueError('Inconsistent PCD field definitions')
        fields = []
        for name, size, kind, count in zip(names, sizes, kinds, counts):
            if count < 1:
                raise ValueError('Invalid PCD field count')
            dtype = {'F': '<f', 'I': '<i', 'U': '<u'}[kind] + str(size)
            fields.append((name, dtype, (count,)) if count > 1 else (name, dtype))
        dtype = np.dtype(fields)
        count = int(header['POINTS'][0])
        if count < 1:
            raise ValueError('Empty PCD')
        if header['DATA'] == ['binary']:
            data = np.fromfile(stream, dtype=dtype, count=count)
        elif header['DATA'] == ['ascii']:
            raw = np.loadtxt(stream, ndmin=2)
            if raw.shape != (count, sum(counts)):
                raise ValueError('PCD payload dimensions do not match header')
            data = np.empty(count, dtype=dtype)
            offset = 0
            for name, width in zip(names, counts):
                data[name] = raw[:, offset] if width == 1 else raw[:, offset:offset + width]
                offset += width
        else:
            raise ValueError('Viewer expects ASCII/binary PCD exported by the backend')
        if len(data) != count:
            raise ValueError('Truncated PCD payload')
    xyz = np.column_stack([data[name] for name in ('x', 'y', 'z')]).astype(float)
    intensity = data['intensity'].astype(float) if 'intensity' in names else None
    if xyz.shape != (count, 3) or not np.isfinite(xyz).all():
        raise ValueError('Invalid XYZ fields')
    if intensity is not None and (intensity.shape != (count,) or not np.isfinite(intensity).all()):
        raise ValueError('Invalid intensity field')
    return xyz, intensity


def read_matrix(path, columns=None):
    values = np.loadtxt(path, ndmin=2)
    if not values.size or not np.isfinite(values).all():
        raise ValueError(f'Empty/nonfinite input: {path}')
    if columns is not None and values.shape[1] != columns:
        raise ValueError(f'Expected {columns} columns: {path}')
    return values


def read_trajectory(path):
    trajectory = read_matrix(path, 8)
    if np.any(np.diff(trajectory[:, 0]) <= 0):
        raise ValueError(f'Timestamps must increase: {path}')
    norms = np.linalg.norm(trajectory[:, 4:], axis=1)
    if np.any(np.abs(norms - 1) > 1e-3):
        raise ValueError(f'Invalid quaternion norm: {path}')
    trajectory[:, 4:] /= norms[:, None]
    return trajectory
