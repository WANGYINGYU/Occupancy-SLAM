"""ROS-independent scan conversion, TF interpolation and scan odometry."""
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
import bisect
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform(translation, rotation):
    q = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid TF quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = [translation.x, translation.y, translation.z]
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite transform')
    return result


def planar(matrix):
    return np.array([matrix[0, 3], matrix[1, 3], math.atan2(matrix[1, 0], matrix[0, 0])])


from geometry import se2, pose2


class Transforms:
    """Offline TF graph; interpolate translations and quaternions at header time."""
    def __init__(self, max_gap=0.2):
        self.static = {}
        self.dynamic = defaultdict(list)
        self.max_gap = max_gap * 1e9
        self.times = {}

    def add(self, parent, child, time, matrix, static=False):
        key = (parent.lstrip('/'), child.lstrip('/'))
        if key[0] == key[1]:
            return
        if static:
            if key in self.static and not np.allclose(self.static[key], matrix, atol=1e-6):
                raise ValueError(f'Conflicting static TF for {key}')
            self.static[key] = matrix
        else:
            self.dynamic[key].append((time, matrix))

    def finish(self):
        for key, rows in self.dynamic.items():
            unique = dict(rows)
            self.dynamic[key] = sorted(unique.items())
            self.times[key] = [row[0] for row in self.dynamic[key]]

    def at(self, key, time):
        if key in self.static:
            return self.static[key]
        times = self.times[key]
        rows = self.dynamic[key]
        index = bisect.bisect_left(times, time)
        if index < len(times) and times[index] == time:
            return rows[index][1]
        if index == 0 or index == len(times):
            raise ValueError(f'TF extrapolation for {key} at {time}')
        t0, m0 = rows[index - 1]
        t1, m1 = rows[index]
        if max(time - t0, t1 - time) > self.max_gap:
            raise ValueError(f'TF/odometry interpolation gap for {key} at {time}')
        alpha = (time - t0) / (t1 - t0)
        m = np.eye(4)
        m[:3, 3] = (1 - alpha) * m0[:3, 3] + alpha * m1[:3, 3]
        m[:3, :3] = Slerp([0., 1.], Rotation.from_matrix([m0[:3, :3], m1[:3, :3]]))([alpha]).as_matrix()[0]
        return m

    def lookup(self, target, source, time):
        target, source = target.lstrip('/'), source.lstrip('/')
        queue = deque([(source, np.eye(4))])
        visited = {source}
        keys = self.static.keys() | self.dynamic.keys()
        while queue:
            frame, accumulated = queue.popleft()
            if frame == target:
                return accumulated
            for parent, child in keys:
                if frame not in (parent, child):
                    continue
                neighbor = parent if frame == child else child
                if neighbor in visited:
                    continue
                try:
                    m = self.at((parent, child), time)
                except ValueError:
                    continue
                if frame == parent:
                    m = np.linalg.inv(m)
                visited.add(neighbor)
                queue.append((neighbor, m @ accumulated))
        raise ValueError(f'No valid TF {target} <- {source} at {time}; supply TF/extrinsics or choose scan-match odometry')


@dataclass
class Scan:
    time: int
    frame: str
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float

    def points(self):
        angles = self.angle_min + np.arange(len(self.ranges)) * self.angle_increment
        good = np.isfinite(self.ranges) & (self.ranges > self.range_min) & (self.ranges < self.range_max)
        return np.column_stack((self.ranges[good] * np.cos(angles[good]), self.ranges[good] * np.sin(angles[good])))


def cloud_xyz(msg):
    # Respect field offsets, row padding, endianness and organized clouds.
    by_name = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = by_name.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError(f'PointCloud2 needs scalar FLOAT32/FLOAT64 {name}')
        formats.append(endian + ('f4' if field.datatype == 7 else 'f8'))
        offsets.append(field.offset)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('Invalid PointCloud2 dimensions/data length')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    return np.column_stack([array[name].ravel() for name in ('x', 'y', 'z')])


def project_points(points, time, frame, args):
    good = np.all(np.isfinite(points), axis=1)
    good &= (points[:, 2] >= args.min_height) & (points[:, 2] <= args.max_height)
    p = points[good]
    radius = np.hypot(p[:, 0], p[:, 1])
    good = (radius > args.min_range) & (radius < args.max_range)
    p, radius = p[good], radius[good]
    angle_min = -math.pi
    increment = 2 * math.pi / args.bins
    indices = np.floor((np.arctan2(p[:, 1], p[:, 0]) - angle_min) / increment + 0.5).astype(int) % args.bins
    ranges = np.full(args.bins, np.inf)
    np.minimum.at(ranges, indices, radius)
    return Scan(time, frame, ranges, angle_min, increment, args.min_range, args.max_range)


def convert_scan(msg, msgtype, graph, args):
    time, frame = stamp_ns(msg.header.stamp), msg.header.frame_id.lstrip('/')
    target = (args.projection_frame or frame).lstrip('/')
    if msgtype.endswith('/LaserScan') and target == frame:
        ranges = np.asarray(msg.ranges, dtype=float).copy()
        increment, angle_min = float(msg.angle_increment), float(msg.angle_min)
        if increment < 0:
            angle_min += increment * (len(ranges) - 1)
            ranges, increment = ranges[::-1], -increment
        scan = Scan(time, frame, ranges, angle_min, increment,
                    max(float(msg.range_min), args.min_range), min(float(msg.range_max), args.max_range))
    else:
        if msgtype.endswith('/PointCloud2'):
            points = cloud_xyz(msg)
        elif msgtype.endswith('/LaserScan'):
            angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
            ranges = np.asarray(msg.ranges, dtype=float)
            good = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            points = np.column_stack([ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good]), np.zeros(np.count_nonzero(good))])
        else:
            raise ValueError(f'Unsupported scan type: {msgtype}')
        if target != frame:
            m = graph.lookup(target, frame, time)
            points = points @ m[:3, :3].T + m[:3, 3]
        scan = project_points(points, time, target, args)
    scan.ranges = scan.ranges[::args.beam_stride]
    scan.angle_increment *= args.beam_stride
    bad = ~np.isfinite(scan.ranges) | (scan.ranges <= scan.range_min) | (scan.ranges >= scan.range_max)
    scan.ranges[bad] = np.inf
    if len(scan.ranges) < 2 or scan.angle_increment <= 0 or scan.range_max <= scan.range_min:
        raise ValueError('Invalid scan geometry/range interval')
    if len(scan.points()) < 20:
        raise ValueError(f'Fewer than 20 valid returns at {time}; check height/range settings')
    return scan


def scan_match(previous, current, guess=None, max_distance=0.8):
    """Trimmed point-to-point SE(2) ICP, used only as slam_toolbox's odom prior."""
    previous = previous[::max(1, len(previous) // 1000)]
    current = current[::max(1, len(current) // 1000)]
    tree = cKDTree(previous)
    result = np.eye(3) if guess is None else guess.copy()
    rmse = math.inf
    for _ in range(40):
        moved = current @ result[:2, :2].T + result[:2, 2]
        distances, ids = tree.query(moved)
        valid = distances < max_distance
        if np.count_nonzero(valid) < 20:
            raise ValueError('Scan odometry has fewer than 20 correspondences; provide odometry or adjust --icp-distance')
        threshold = np.quantile(distances[valid], 0.85)
        valid &= distances <= max(threshold, 0.02)
        source, destination = moved[valid], previous[ids[valid]]
        x0, y0 = source.mean(axis=0), destination.mean(axis=0)
        u, _, vt = np.linalg.svd((source - x0).T @ (destination - y0))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        delta = np.eye(3)
        delta[:2, :2], delta[:2, 2] = rotation, y0 - rotation @ x0
        result = delta @ result
        rmse = float(np.sqrt(np.mean(distances[valid] ** 2)))
        if np.linalg.norm(delta[:2, 2]) < 1e-5 and abs(math.atan2(delta[1, 0], delta[0, 0])) < 1e-5:
            break
    return result, rmse


def bag_inputs(args):
    from rosbags.highlevel import AnyReader
    bag = Path(args.bag).expanduser().resolve()
    graph = Transforms(args.tf_max_gap)
    odom_graph = Transforms(args.tf_max_gap)
    with AnyReader([bag]) as reader:
        candidates = [c for c in reader.connections if c.msgtype in ('sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/PointCloud2')]
        topics = sorted({c.topic for c in candidates})
        if args.scan_topic == 'auto':
            if len(topics) != 1:
                raise ValueError(f'Set --scan-topic; available scan topics: {topics}')
            args.scan_topic = topics[0]
        selected = [c for c in candidates if c.topic.lstrip('/') == args.scan_topic.lstrip('/')]
        if not selected:
            raise ValueError(f'No scan topic {args.scan_topic}; choices: {topics}')
        if len({c.msgtype for c in selected}) != 1:
            raise ValueError(f'Mixed message types on scan topic {args.scan_topic}')
        args.scan_topic = selected[0].topic
        tf_topic = args.tf_topic.lstrip('/')
        static_topic = args.tf_static_topic.lstrip('/')
        args.resolved_input_topics = dict(scan=args.scan_topic, scan_type=selected[0].msgtype,
                                         odometry=args.odom_topic, tf=args.tf_topic, tf_static=args.tf_static_topic)
        odom_connections = [c for c in reader.connections if c.topic.lstrip('/') == args.odom_topic.lstrip('/') and c.msgtype == 'nav_msgs/msg/Odometry']
        if args.odom_source == 'odom' and not odom_connections:
            available = sorted({c.topic for c in reader.connections if c.msgtype == 'nav_msgs/msg/Odometry'})
            raise ValueError(f'No Odometry on {args.odom_topic}; available odometry topics: {available}')
        tf_connections = [c for c in reader.connections if c.topic.lstrip('/') in (tf_topic, static_topic)]
        wrong = [c.topic for c in tf_connections if c.msgtype != 'tf2_msgs/msg/TFMessage']
        if wrong:
            raise ValueError(f'Configured TF topics must have TFMessage type: {wrong}')
        connections = tf_connections + odom_connections
        for connection, _, raw in reader.messages(connections=connections) if connections else []:
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic.lstrip('/') == args.odom_topic.lstrip('/'):
                p = msg.pose.pose
                odom_graph.add(msg.header.frame_id, msg.child_frame_id, stamp_ns(msg.header.stamp), transform(p.position, p.orientation))
            else:
                for tf in msg.transforms:
                    graph.add(tf.header.frame_id, tf.child_frame_id, stamp_ns(tf.header.stamp), transform(tf.transform.translation, tf.transform.rotation), connection.topic.lstrip('/') == static_topic)
        graph.finish()
        # An odometry message provides only odom->child; sensor mounting remains from TF.
        odom_graph.static.update(graph.static)
        for key, value in graph.dynamic.items():
            if key not in odom_graph.dynamic:
                odom_graph.dynamic[key] = value
        odom_graph.finish()
        scans = []
        for i, (connection, _, raw) in enumerate(reader.messages(connections=selected)):
            if i < args.start or (i - args.start) % args.scan_stride:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            scan = convert_scan(msg, connection.msgtype, graph, args)
            if scans and scan.time <= scans[-1].time:
                raise ValueError('Scan header timestamps must strictly increase')
            scans.append(scan)
            if args.max_scans and len(scans) >= args.max_scans:
                break
    if len(scans) < 2:
        raise ValueError('Need at least two scans')
    reference = scans[0]
    for scan in scans:
        if scan.frame != reference.frame or len(scan.ranges) != len(reference.ranges) or not np.allclose([scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max], [reference.angle_min, reference.angle_increment, reference.range_min, reference.range_max]):
            raise ValueError('Changing scan geometry/frame; split the bag or use --projection-frame with a fixed angular grid')
    source = args.odom_source
    if source == 'auto':
        if odom_connections:
            source = 'odom'
        else:
            try:
                graph.lookup(args.odom_frame, scans[0].frame, scans[0].time)
                source = 'tf'
            except ValueError:
                source = 'scan-match'
    priors, diagnostics = [], []
    if source in ('tf', 'odom'):
        selected_graph = graph if source == 'tf' else odom_graph
        priors = [planar(selected_graph.lookup(args.odom_frame, s.frame, s.time)) for s in scans]
    else:
        pose, previous_delta = np.eye(3), np.eye(3)
        priors.append(pose2(pose))
        for i in range(1, len(scans)):
            delta, error = scan_match(scans[i-1].points(), scans[i].points(), previous_delta, args.icp_distance)
            pose = pose @ delta
            previous_delta = delta
            priors.append(pose2(pose))
            diagnostics.append({'time_ns': scans[i].time, 'trimmed_icp_rmse_m': error})
    return scans, np.asarray(priors), source, diagnostics
