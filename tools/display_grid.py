"""Independent-resolution 2D display integration using the backend hit/free model.

No optimizer state is changed. Sampling follows FuncConvertObs at the requested
display resolution; accumulation follows FuncInitialiseGridMap floor bins.
"""
import math
import numpy as np

HIT_LOG_ODDS = 0.847297860387203
FREE_LOG_ODDS = -0.405465108108164


def scan_samples(scan, angles, config, resolution):
    if scan.ndim == 2 and scan.shape[1] == 2:
        hit = scan
        distance = np.linalg.norm(hit, axis=1)
    else:
        distance = scan
        hit = np.column_stack((distance*np.cos(angles), distance*np.sin(angles)))
    valid = np.isfinite(hit).all(axis=1) & (distance > float(config['MinRange'])) & (distance < float(config['MaxRange']))
    distance, hit = distance[valid], hit[valid]
    counts = np.floor(distance/resolution).astype(int)
    hit, counts = hit[counts > 0], counts[counts > 0]
    if not len(counts):
        return np.empty((0, 2)), np.empty(0, dtype=bool)
    sample_index = np.arange(counts.sum()) - np.repeat(np.cumsum(counts)-counts, counts)
    local = np.repeat(hit/counts[:, None], counts, axis=0)*(np.repeat(counts, counts)-sample_index)[:, None]
    endpoints = sample_index == 0
    local[endpoints] = hit
    return local, endpoints


def shared_geometry(native_grid, point_sets, resolution):
    """Cover the native field and both trajectories/scans on one oriented lattice."""
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Display resolution must be finite and positive')
    x, y, yaw = native_grid['origin']
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    height, width = native_grid['values'].shape
    corners = np.array([[0, 0], [width, 0], [0, height], [width, height]])*native_grid['resolution']
    local = [corners] + [(points[:, :2]-[x,y]) @ rotation for points in point_sets if len(points)]
    lower = np.floor(np.min([points.min(axis=0) for points in local], axis=0)/resolution)-1
    upper = np.floor(np.max([points.max(axis=0) for points in local], axis=0)/resolution)+2
    dims = upper-lower
    if not np.isfinite(dims).all() or np.prod(dims) > 10_000_000:
        raise ValueError('Display grid exceeds 10M cells; increase --display-resolution')
    origin = lower*resolution @ rotation.T + [x,y]
    return dict(resolution=resolution, origin=[*map(float,origin),yaw],
                shape=(int(dims[1]),int(dims[0])))


def integrate(ranges, poses, config, geometry, hit_log_odds=HIT_LOG_ODDS, free_log_odds=FREE_LOG_ODDS):
    """Replay scans at fixed poses into an empty grid; never resample an old map."""
    if not math.isfinite(hit_log_odds) or not math.isfinite(free_log_odds) or hit_log_odds <= 0 or free_log_odds >= 0:
        raise ValueError('Invalid hit/free log odds')
    resolution = geometry['resolution']
    height, width = geometry['shape']
    if len(ranges) != len(poses):
        raise ValueError('Display integration needs matching scans and poses')
    x, y, yaw = geometry['origin']
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    log_odds = np.zeros((height,width),dtype=float)
    counts = np.zeros((height,width),dtype=np.uint64)
    angles = np.linspace(float(config['MinAngle']),float(config['MaxAngle']),ranges.shape[1])
    for scan, pose in zip(ranges,poses):
        local, endpoints = scan_samples(scan,angles,config,resolution)
        if not len(local):
            continue
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = local @ np.array([[c,s],[-s,c]]) + pose[:2]
        index = np.floor(((world-[x,y]) @ rotation)/resolution).astype(int)
        ix, iy = index.T
        if np.any((ix < 0) | (ix >= width) | (iy < 0) | (iy >= height)):
            raise ValueError('Display grid does not cover all hit/free observations')
        flat = iy*width+ix
        scan_sum = np.bincount(flat,weights=np.where(endpoints,hit_log_odds,free_log_odds),
                               minlength=height*width).reshape(height,width)
        scan_count = np.bincount(flat,minlength=height*width).reshape(height,width)
        log_odds += scan_sum
        counts += scan_count.astype(np.uint64)
    return log_odds, counts
