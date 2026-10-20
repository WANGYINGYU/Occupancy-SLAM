"""Read the original Simu*_Data MATLAB v5/v7.3 datasets without angle resampling."""
from pathlib import Path
import numpy as np
from scipy.io import loadmat


def is_hdf(path):
    with Path(path).open('rb') as f:
        return b'MATLAB 7.3' in f.read(128)


def numeric_variables(path, keys):
    if is_hdf(path):
        import h5py
        with h5py.File(path,'r') as f: return {k:f[k][()].T for k in keys}
    d=loadmat(path,variable_names=keys)
    return {k:d[k] for k in keys}


def scan_cells(path):
    if is_hdf(path):
        import h5py
        with h5py.File(path,'r') as f:
            for reference in f['Scan'][()].T.ravel():
                group=f[reference]
                yield group['xy'][()].T,group['Odd'][()].T.ravel()
    else:
        cells=loadmat(path,variable_names=['Scan'],simplify_cells=True)['Scan']
        for cell in cells: yield np.asarray(cell['xy']),np.asarray(cell['Odd']).ravel()


def raw_scans(path, indices):
    if is_hdf(path):
        import h5py
        with h5py.File(path,'r') as f:
            refs=f['scan'][()].T.ravel()
            return [f[refs[i]][()].T for i in indices]
    cells=loadmat(path,variable_names=['scan'])['scan'].ravel()
    return [cells[i] for i in indices]


def extract_endpoints(path):
    scans=[]; counts=[]; max_replay_error=0.; hit_value=None; free_value=None
    for xy,odd in scan_cells(path):
        if xy.ndim!=2 or xy.shape[1]!=2 or len(xy)!=len(odd) or not np.isfinite(xy).all() or not np.isfinite(odd).all():
            raise ValueError('Invalid MATLAB Scan.xy/Odd')
        starts=np.flatnonzero(odd>0)
        if not len(starts) or starts[0]!=0: raise ValueError('Scan must begin with a hit')
        hits=xy[starts]; lengths=np.diff(np.r_[starts,len(xy)])
        if not np.allclose(odd[starts],.847297860387203,atol=1e-14,rtol=0) or not np.allclose(odd[odd<0],-.405465108108164,atol=1e-14,rtol=0):
            raise ValueError('Unexpected MATLAB occupancy labels')
        # Validate all stored free samples, not only endpoints, against their ray.
        k=np.arange(len(xy))-np.repeat(starts,lengths)
        rebuilt=np.repeat(hits/lengths[:,None],lengths,axis=0)*(np.repeat(lengths,lengths)-k)[:,None]
        rebuilt[starts]=hits
        max_replay_error=max(max_replay_error,float(np.max(np.abs(rebuilt-xy))))
        scans.append(hits);counts.append(dict(rays=len(hits),stored_samples=len(xy),
            sample_resolution_median=float(np.median(np.linalg.norm(hits,axis=1)/lengths))))
    if len({len(x) for x in scans})!=1: raise ValueError('Expected equal beam counts for these datasets')
    return np.stack(scans),dict(frames=len(scans),per_frame=counts,max_stored_free_sample_replay_error_m=max_replay_error)
