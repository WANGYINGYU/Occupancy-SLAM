"""ROS-free result reader and display-map reconstruction."""
import math,json
from pathlib import Path
import numpy as np
import yaml
from result_io import read_matrix,read_pcd,read_trajectory
from result_layout import ResultLayout
from backend_config import read_parameters
from display_grid import scan_samples,shared_geometry,integrate,HIT_LOG_ODDS,FREE_LOG_ODDS
HEIGHT_COLORS = np.array([[36,93,170],[40,171,202],[114,207,184],[250,225,130],[244,144,78],[205,62,62]],dtype=float)
def se2_trajectory(poses, times):
    rows = np.zeros((len(poses), 8))
    rows[:, 0], rows[:, 1:3] = times, poses[:, :2]
    rows[:, 6], rows[:, 7] = np.sin(poses[:, 2] / 2), np.cos(poses[:, 2] / 2)
    return rows


def scan_points(ranges, poses, config):
    if ranges.ndim == 3:
        clouds=[]
        for local,pose in zip(ranges,poses):
            distance=np.linalg.norm(local,axis=1)
            good=np.isfinite(local).all(axis=1)&(distance>float(config['MinRange']))&(distance<float(config['MaxRange']))
            c,s=np.cos(pose[2]),np.sin(pose[2])
            world=local[good] @ np.array([[c,s],[-s,c]])+pose[:2]
            clouds.append(np.column_stack([world,np.zeros(len(world))]))
        return np.concatenate(clouds)
    angles = np.linspace(float(config['MinAngle']), float(config['MaxAngle']), ranges.shape[1])
    valid = (ranges > float(config['MinRange'])) & (ranges < float(config['MaxRange']))
    x, y = ranges * np.cos(angles), ranges * np.sin(angles)
    c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
    wx = c*x - s*y + poses[:, 0, None]
    wy = s*x + c*y + poses[:, 1, None]
    return np.column_stack((wx[valid], wy[valid], np.zeros(np.count_nonzero(valid))))


def load_grid(folder):
    meta = yaml.safe_load((folder/'map.yaml').read_text())
    resolution = float(meta['resolution'])
    origin = np.asarray(meta['origin'], dtype=float)
    if not math.isfinite(resolution) or resolution <= 0 or origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('Invalid map resolution/origin')
    raw = read_matrix(folder/'occupancy.txt')
    count = read_matrix(folder/'normalization.txt')
    if raw.shape != count.shape or np.any(count < 0):
        raise ValueError('Map/normalization shape or value mismatch')
    probability = 1 / (1 + np.exp(-np.clip(raw / np.maximum(count, 1e-9), -30, 30)))
    # Backend matrices already run from low y to high y. PGM image flips do NOT apply.
    values = np.rint(100 * probability).astype(np.int8)
    return dict(resolution=resolution, origin=origin.tolist(), values=values, accumulated_log_odds=raw)


def height_rgb(z, lower, upper):
    """Pack a shared world-height color into standard PointCloud2 rgb UINT32."""
    unit = np.clip((np.asarray(z)-lower)/(upper-lower), 0, 1)
    positions = np.linspace(0, 1, len(HEIGHT_COLORS))
    colors = np.column_stack([np.interp(unit, positions, HEIGHT_COLORS[:, c]) for c in range(3)])
    colors = np.rint(colors).astype(np.uint32)
    return (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]


def observation_support(ranges, poses, config, grid, bilinear=False):
    """Actual hit/free sample support; never infer observation coverage from smoothed N."""
    scale = grid['resolution']
    height, width = grid['values'].shape
    support = np.zeros((height, width), dtype=bool)
    angles = np.linspace(float(config['MinAngle']), float(config['MaxAngle']), ranges.shape[1])
    origin = np.asarray(grid['origin'][:2])
    yaw = grid['origin'][2]
    inverse_grid_rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    for scan, pose in zip(ranges, poses):
        local, _ = scan_samples(scan, angles, config, scale)
        if not len(local):
            continue
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        world = local @ np.array([[c, sn], [-sn, c]]) + pose[:2]
        query = ((world-origin) @ inverse_grid_rotation)/scale
        base = np.floor(query).astype(int)
        frac = query-base
        offsets = ((0, 0), (1, 0), (0, 1), (1, 1)) if bilinear else ((0, 0),)
        for dx, dy in offsets:
            x, y = base[:, 0]+dx, base[:, 1]+dy
            valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            if bilinear:
                weight = (frac[:, 0] if dx else 1-frac[:, 0])*(frac[:, 1] if dy else 1-frac[:, 1])
                valid &= weight > 0
            support[y[valid], x[valid]] = True
    return support


def classified_grid(log_odds, observed, free_threshold=.35, occupied_threshold=.65):
    # Upstream FuncShowMap renders sigmoid(M), whereas the residual uses M/N.
    # This is a separate display topic; the original normalized probabilities stay intact.
    probability = 1/(1+np.exp(-np.clip(log_odds, -30, 30)))
    result = np.full(log_odds.shape, -1, dtype=np.int8)
    result[observed & (probability <= free_threshold)] = 0
    result[observed & (probability >= occupied_threshold)] = 100
    return result


def load_scene(args):
    result, initial = args.result, args.initial
    status_path=result/'run_status.json'
    if status_path.exists():
        status=json.loads(status_path.read_text())['status']
        if status in ('failed','interrupted') or (status=='running' and not getattr(args,'allow_running',False)):raise ValueError(f'Result is {status}: {result}')
    layout=ResultLayout.read(result/'optimized' if args.mode=='2d' else result)
    scene = dict(paths={}, clouds={}, maps={}, mode=args.mode)
    if args.mode == '2d':
        before = read_matrix(result/'Pose.txt', 3)
        after = read_matrix(layout.joint/'poses.txt', 3)
        times = read_matrix(result/'timestamps.txt', 1).ravel()
        if (result/'Endpoints.npz').exists():
            with np.load(result/'Endpoints.npz',allow_pickle=False) as data: ranges=data['endpoints']
            if ranges.ndim!=3 or ranges.shape[2]!=2 or not np.isfinite(ranges).all(): raise ValueError('Invalid endpoint scans')
        else:
            ranges = read_matrix(result/'Range.txt')
        if before.shape != after.shape or len(before) != len(times) or len(ranges) != len(times):
            raise ValueError('Matched scans/poses/timestamps differ in length')
        if np.any(np.diff(times) <= 0):
            raise ValueError('Scan timestamps must increase')
        config = read_parameters(result/'backend_config.txt')
        if ranges.shape[1] != int(config['NumBeam']):
            raise ValueError('Scan columns differ from NumBeam')
        for name, poses in [('initial', before), ('optimized', after)]:
            scene['paths'][name] = se2_trajectory(poses, times)
            scene['clouds'][name + '/points'] = (scan_points(ranges, poses, config), None)
        scene['maps']['optimized'] = load_grid(layout.joint)
        if initial is None and layout.initial.is_dir():
            initial = layout.initial
        if layout.reintegrated.is_dir():
            scene['maps']['reintegrated'] = load_grid(layout.reintegrated)
            replay_poses = read_matrix(layout.reintegrated/'poses.txt', 3)
            if not np.array_equal(replay_poses, after):
                raise ValueError('Reintegration must use the exact joint poses')
            scene['paths']['reintegrated'] = scene['paths']['optimized'].copy()
            scene['clouds']['reintegrated/points'] = scene['clouds']['optimized/points']
        if initial:
            check = read_matrix(initial/'poses.txt', 3)
            if check.shape != before.shape or not np.allclose(check, before, atol=1e-9, rtol=0):
                raise ValueError('Initial map poses differ from matched pipeline poses')
            scene['maps']['initial'] = load_grid(initial)
    else:
        scene['paths']['optimized'] = read_trajectory(layout.joint/'trajectory.tum')
        scene['clouds']['optimized/points'] = read_pcd(layout.joint/'optimized_points.pcd')
        scene['clouds']['optimized/occupancy'] = read_pcd(layout.joint/'occupancy_vertices.pcd')
        if initial is None and layout.initial.is_dir():
            initial = layout.initial
        if layout.reintegrated.is_dir():
            scene['paths']['reintegrated'] = read_trajectory(layout.reintegrated/'trajectory.tum')
            if not np.array_equal(scene['paths']['reintegrated'], scene['paths']['optimized']):
                raise ValueError('Reintegration must use the exact joint poses')
            scene['clouds']['reintegrated/points'] = read_pcd(layout.reintegrated/'optimized_points.pcd')
            scene['clouds']['reintegrated/occupancy'] = read_pcd(layout.reintegrated/'occupancy_vertices.pcd')
        if initial:
            before = read_trajectory(initial/'trajectory.tum')
            after = scene['paths']['optimized']
            if before.shape != after.shape or not np.allclose(before[:, 0], after[:, 0], atol=1e-7, rtol=0):
                raise ValueError('Initial and optimized trajectory timestamps differ')
            if (initial/'selected_scans.txt').read_text() != (layout.joint/'selected_scans.txt').read_text():
                raise ValueError('Initial and optimized results select different scans')
            scene['paths']['initial'] = before
            scene['clouds']['initial/points'] = read_pcd(initial/'optimized_points.pcd')
            scene['clouds']['initial/occupancy'] = read_pcd(initial/'occupancy_vertices.pcd')
    if args.mode == '2d' and getattr(args,'display_maps',None):
        folder=Path(args.display_maps)
        description=json.loads((folder/'display_manifest.json').read_text())
        if description['resolution'] != args.display_resolution: raise ValueError('Display export resolution differs from requested resolution')
        for name,grid in scene['maps'].items():
            meta=yaml.safe_load((folder/name/'map.yaml').read_text())
            with np.load(folder/name/'grid.npz',allow_pickle=False) as data:
                grid['display_grid']=dict(resolution=meta['resolution'],origin=meta['origin'],**{k:data[k] for k in data.files})
            grid['display_stats']=description['methods'][name]
        scene['display_integration']=description['integration']
    elif args.mode == '2d':
        geometry = shared_geometry(scene['maps']['optimized'],
                                   [cloud[0] for cloud in scene['clouds'].values()] +
                                   [path[:,1:3] for path in scene['paths'].values()], args.display_resolution)
        metadata = result/'optimized/map_outputs.json'
        model = json.loads(metadata.read_text()) if metadata.exists() else {}
        hit = float(model.get('hit_log_odds', HIT_LOG_ODDS))
        free = float(model.get('free_log_odds', FREE_LOG_ODDS))
        for name, grid in scene['maps'].items():
            if name == 'optimized':
                support = observation_support(ranges, after, config, grid, bilinear=True)
                raw = grid['accumulated_log_odds']
                display_grid = dict(resolution=grid['resolution'], origin=grid['origin'])
                source = 'joint optimization variables on their native grid'
                support_source = 'hit/free sample bilinear support'
            else:
                raw, counts = integrate(ranges, before if name == 'initial' else after,
                                        config, geometry, hit, free)
                support = counts > 0
                display_grid = dict(resolution=geometry['resolution'], origin=geometry['origin'],
                                    integration_counts=counts, accumulated_log_odds=raw)
                source = 'fresh fixed-pose hit/free accumulation from '+('Endpoints.npz' if ranges.ndim==3 else 'Range.txt')
                support_source = 'display-resolution hit/free floor bins'
            display_grid['values'] = classified_grid(raw, support)
            grid['display_grid'] = display_grid
            grid['display_stats'] = dict(source=source, support_source=support_source,
                                         observed_cells=int(support.sum()),
                                         free_cells=int((display_grid['values'] == 0).sum()),
                                         occupied_cells=int((display_grid['values'] == 100).sum()),
                                         unknown_cells=int((display_grid['values'] == -1).sum()))
        scene['display_integration'] = dict(resolution=args.display_resolution,
            hit_log_odds=hit, free_log_odds=free, shared_geometry=geometry,
            sampling='FuncConvertObs: floor(range / display_resolution) samples, endpoint hit, remaining free',
            binning='floor', poses='fixed input/final poses', resampled_optimizer_field=False)
    else:
        reference = 'initial/points' if 'initial/points' in scene['clouds'] else 'optimized/points'
        heights = scene['clouds'][reference][0][:, 2]
        limits = list(args.height_range) if args.height_range else np.quantile(heights, [.01, .99]).tolist()
        if limits[1]-limits[0] < 1e-6:
            limits = [limits[0]-.5, limits[1]+.5]
        scene['height_color'] = dict(channel='z', min=limits[0], max=limits[1],
                                    palette='blue-cyan-yellow-orange-red', rgb_anchors=HEIGHT_COLORS.astype(int).tolist(),
                                    source='explicit' if args.height_range else reference+' percentiles 1,99',
                                    shared_across_methods=True, outside_range='color clamped; points retained')
    scene['initial_source'] = str(initial) if initial else None
    stats = {}
    for name, (xyz, values) in list(scene['clouds'].items()):
        raw_count = len(xyz)
        if name.endswith('/occupancy'):
            if values is None or np.any((values < 0) | (values > 1)):
                raise ValueError('Occupancy PCD needs intensity probabilities in [0, 1]')
            mask = values >= args.occupancy_threshold
            xyz, values = xyz[mask], values[mask]
        filtered_count = len(xyz)
        if args.max_points and len(xyz) > args.max_points:
            ids = np.linspace(0, len(xyz)-1, args.max_points, dtype=int)
            xyz = xyz[ids]
            values = values[ids] if values is not None else None
        scene['clouds'][name] = xyz, values
        stats[name] = dict(source_points=raw_count, threshold_points=filtered_count, published_points=len(xyz))
    # Fit observed points and complete map extents, including the saved field halo.
    fit = [xyz for name, (xyz, _) in scene['clouds'].items() if name.endswith('/points') and len(xyz)]
    fit += [poses[:, 1:4] for poses in scene['paths'].values()]
    for grid in [g for native in scene['maps'].values() for g in (native, native['display_grid'])]:
        height, width = grid['values'].shape
        corners = np.array([[0, 0], [width, 0], [0, height], [width, height]]) * grid['resolution']
        x, y, yaw = grid['origin']
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        fit.append(np.column_stack((corners @ rotation.T + [x, y], np.zeros(4))))
    lo = np.min([v.min(axis=0) for v in fit], axis=0)
    hi = np.max([v.max(axis=0) for v in fit], axis=0)
    scene['bounds'] = [lo.tolist(), hi.tolist()]
    scene['stats'] = stats
    return scene


def save_display_maps(scene, output, nested=True):
    """Archive the actual published high-resolution grids with ROS map metadata."""
    for method, native in scene['maps'].items():
        grid = native['display_grid']
        folder = (output/'display_maps' if nested else output)/method
        folder.mkdir(parents=True)
        values = grid['values']
        # PGM's first row is the highest y; ROS message rows run from low to high y.
        image = np.where(values == -1, 128, np.where(values == 100, 0, 255)).astype(np.uint8)[::-1]
        (folder/'map.pgm').write_bytes(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode()+image.tobytes())
        meta = dict(image='map.pgm', resolution=grid['resolution'], origin=grid['origin'],
                    negate=0, occupied_thresh=.65, free_thresh=.35, mode='trinary')
        (folder/'map.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
        payload = {key: grid[key] for key in ('values', 'accumulated_log_odds', 'integration_counts') if key in grid}
        np.savez_compressed(folder/'grid.npz', **payload)
