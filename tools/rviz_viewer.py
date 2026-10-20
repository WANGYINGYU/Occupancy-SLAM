#!/usr/bin/env python3
"""Publish saved Occupancy-SLAM results and open a fitted ROS2 RViz view."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from result_io import read_matrix, read_pcd, read_trajectory
from result_scene import load_scene, load_grid, scan_points, observation_support, classified_grid, height_rgb, save_display_maps
from export_maps import prepare_display

ROOT = Path(__file__).resolve().parents[1]
# A restrained height palette: blue, cyan, pale yellow, orange, red.
HEIGHT_COLORS = np.array([[36, 93, 170], [40, 171, 202], [114, 207, 184],
                          [250, 225, 130], [244, 144, 78], [205, 62, 62]], dtype=float)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=('2d', '3d'))
    p.add_argument('--result', type=Path, required=True,
                   help='2D pipeline directory (Pose.txt + optimized/), or 3D output directory')
    p.add_argument('--initial', type=Path,
                   help='Optional matching --iterations 0 backend result for map/cloud comparison')
    p.add_argument('--output', type=Path, help='Viewer config, manifest and RViz log directory')
    p.add_argument('--frame', default='map', help='Common world frame for saved results')
    p.add_argument('--namespace', default='/occupancy_slam')
    p.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '88')))
    p.add_argument('--max-points', type=int, default=200000,
                   help='Per-cloud deterministic display cap; 0 publishes all points')
    p.add_argument('--occupancy-threshold', type=float, default=0.6,
                   help='3D probability threshold; grid vertices, not surface cells')
    p.add_argument('--method', choices=('initial', 'optimized', 'reintegrated'),
                   help='Initially selected topic; default: 2D reintegrated, 3D optimized')
    p.add_argument('--height-range', type=float, nargs=2, metavar=('MIN_Z', 'MAX_Z'),
                   help='Shared 3D height color limits in world metres; default: initial scan 1st/99th percentiles')
    p.add_argument('--map-view', choices=('classified', 'probability'), default='classified',
                   help='2D: classified upstream log-odds display, or unchanged normalized probability')
    p.add_argument('--display-resolution', type=float, default=.05, metavar='METRES',
                   help='2D initial/reintegrated display grid (default: 0.05 m); optimizer grid stays unchanged')
    cloud_view = p.add_mutually_exclusive_group()
    cloud_view.add_argument('--show-points', dest='cloud_view', action='store_const', const='points', default='points',
                            help='3D: show scan point clouds (default)')
    cloud_view.add_argument('--show-occupancy', dest='cloud_view', action='store_const', const='occupancy',
                            help='3D: explicitly inspect occupancy vertices')
    p.add_argument('--no-rviz', action='store_true', help='Only publish topics, for a separate RViz instance')
    p.add_argument('--duration', type=float, default=0, help='Exit after N seconds; 0 waits for window close/Ctrl+C')
    return p


def make_config(scene, args):
    config = yaml.safe_load((ROOT/'config'/f'occupancy_{args.mode}.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = args.frame
    kind = ('map_display' if args.map_view == 'classified' else 'map') if args.mode == '2d' else args.cloud_view
    available = {f'{key}/trajectory' for key in scene['paths']}
    available |= {f'{key}/{suffix}' for key in scene['maps'] for suffix in ('map', 'map_display')} | set(scene['clouds'])
    if args.method+'/'+kind not in available:
        raise ValueError(f'Selected result has no {args.method}/{kind} topic')
    displays = []
    for display in manager['Displays']:
        topic = display.get('Topic')
        if isinstance(topic, dict):
            relative = topic['Value'].removeprefix('/occupancy_slam/').replace('optimized/', args.method+'/')
            if args.mode == '2d' and display['Name'] == 'Map':
                relative = args.method+'/'+kind
            if args.mode == '3d' and display['Name'] == 'PointCloud2':
                relative = args.method+'/'+args.cloud_view
            if relative not in available:
                continue
            topic['Value'] = args.namespace + '/' + relative
            if args.mode == '3d' and display['Class'] == 'rviz_default_plugins/PointCloud2':
                display.update({'Color Transformer': 'RGB8', 'Channel Name': 'z',
                                'Autocompute Intensity Bounds': False,
                                'Min Intensity': scene['height_color']['min'],
                                'Max Intensity': scene['height_color']['max']})
        displays.append(display)
    manager['Displays'] = displays
    lo, hi = np.asarray(scene['bounds'])
    center, size = (lo+hi)/2, np.maximum(hi-lo, 1)
    view = manager['Views']['Current']
    if args.mode == '2d':
        view.update(X=float(center[0]), Y=float(center[1]), Scale=float(min(900/size[0], 680/size[1]) * .9))
    else:
        view.update({'Distance': float(np.linalg.norm(size) * 1.35),
                     'Focal Point': dict(zip(('X', 'Y', 'Z'), map(float, center)))})
    return config


def make_messages(scene, frame, stamp):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as PathMessage
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker, MarkerArray
    header = Header(frame_id=frame, stamp=stamp)
    messages = {}
    for name, rows in scene['paths'].items():
        path = PathMessage(header=header)
        for row in rows:
            pose = PoseStamped(header=header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, row[1:4])
            q = pose.pose.orientation
            q.x, q.y, q.z, q.w = map(float, row[4:8])
            path.poses.append(pose)
        messages[name+'/trajectory'] = path
    for name, (xyz, intensity) in scene['clouds'].items():
        fields = ['x', 'y', 'z'] + (['intensity'] if intensity is not None else [])
        points = xyz if intensity is None else np.column_stack((xyz, intensity))
        payload = np.ascontiguousarray(points, dtype='<f4')
        if scene['mode'] == '3d':
            colored = np.empty((len(points), len(fields)+1), dtype='<f4')
            colored[:, :-1] = payload
            color = scene['height_color']
            colored.view('<u4')[:, -1] = height_rgb(xyz[:, 2], color['min'], color['max'])
            payload = colored
            fields.append('rgb')
        cloud = PointCloud2(header=header, height=1, width=len(points), is_dense=True,
                           is_bigendian=False, point_step=4*len(fields), row_step=payload.nbytes,
                           fields=[PointField(name=n, offset=4*i, datatype=PointField.UINT32 if n == 'rgb' else PointField.FLOAT32, count=1)
                                   for i, n in enumerate(fields)], data=payload.tobytes())
        messages[name] = cloud
    for name, grid in scene['maps'].items():
        for suffix, selected in (('map', grid), ('map_display', grid['display_grid'])):
            values = selected['values']
            msg = OccupancyGrid(header=header)
            msg.info.map_load_time = stamp
            msg.info.resolution = selected['resolution']
            msg.info.height, msg.info.width = values.shape
            x, y, yaw = selected['origin']
            msg.info.origin.position.x, msg.info.origin.position.y = x, y
            msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            msg.data = values.ravel().tolist()
            messages[name+'/'+suffix] = msg
    markers = MarkerArray()
    for name, rows in scene['paths'].items():
        for index, (row, label) in enumerate(((rows[0], 'start'), (rows[-1], 'end'))):
            marker = Marker(header=header, ns=name, id=index, type=Marker.SPHERE, action=Marker.ADD)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, row[1:4])
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .22 if scene['mode'] == '2d' else .4
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = ((.15, .85, .45, 1.) if label == 'start' else (1., .25, .25, 1.))
            markers.markers.append(marker)
    messages['landmarks'] = markers
    return messages



def run(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    if args.mode == '2d' and not args.no_rviz:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
        try:
            get_package_prefix('occupancy_slam_rviz')
        except PackageNotFoundError as exc:
            raise RuntimeError('Build the gray Map display with bash tools/build_rviz.sh, then reopen the viewer') from exc
    if args.mode == '2d': args.display_maps=prepare_display(args.result,args.display_resolution,args.output)
    scene = load_scene(args)
    if args.method is None:
        args.method = 'reintegrated' if args.mode == '2d' and 'reintegrated' in scene['maps'] else 'optimized'
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    save_display_maps(scene, out)
    config = out/f'occupancy_{args.mode}.rviz'
    config.write_text(yaml.safe_dump(make_config(scene, args), sort_keys=False))
    rclpy.init(args=[])
    node = Node('occupancy_result_viewer', namespace=args.namespace)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    child = None
    log = None
    started = time.monotonic()
    status = 'completed'
    try:
        messages = make_messages(scene, args.frame, node.get_clock().now().to_msg())
        publishers = {name: node.create_publisher(type(msg), args.namespace+'/'+name, qos)
                      for name, msg in messages.items()}
        # One publish per topic. Transient-local history must serve late RViz subscribers.
        for name, msg in messages.items():
            publishers[name].publish(msg)
        # Register the common world frame without inventing a robot/odometry TF.
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        reference = TransformStamped()
        reference.header.frame_id = args.frame
        reference.header.stamp = node.get_clock().now().to_msg()
        reference.child_frame_id = args.namespace.strip('/').replace('/', '_') + '_display_origin'
        if reference.child_frame_id == args.frame:
            reference.child_frame_id += '_reference'
        reference.transform.rotation.w = 1.
        static_tf = StaticTransformBroadcaster(node)
        static_tf.sendTransform(reference)
        def ros_type(message):
            return type(message).__module__.split('.')[0]+'/msg/'+type(message).__name__
        manifest = dict(mode=args.mode, result=str(args.result), initial=scene['initial_source'],
                        selected_method=args.method, display='native_topic_selection', map_view=args.map_view,
                        cloud_view=args.cloud_view, unknown_color_rgb=[128,128,128],
                        height_color=scene.get('height_color'),
                        display_integration=scene.get('display_integration'),
                        display_maps={k: dict(width=v['display_grid']['values'].shape[1],
                                              height=v['display_grid']['values'].shape[0],
                                              resolution=v['display_grid']['resolution'],
                                              origin=v['display_grid']['origin'],
                                              files=str(out/'display_maps'/k),
                                              **v['display_stats']) for k,v in scene['maps'].items()},
                        map_display=dict(source='sigmoid(M), upstream accumulated log odds', free_threshold=.35, occupied_threshold=.65,
                                         values={'unknown_or_undecided': -1, 'free': 0, 'occupied': 100},
                                         methods={k: v['display_stats'] for k, v in scene['maps'].items()}),
                        frame=args.frame, ros_domain_id=args.domain_id,
                        topics={args.namespace+'/'+n: ros_type(m) for n, m in messages.items()},
                        clouds=scene['stats'], trajectories={k: len(v) for k, v in scene['paths'].items()},
                        maps={k: dict(width=v['values'].shape[1], height=v['values'].shape[0], resolution=v['resolution'], origin=v['origin']) for k, v in scene['maps'].items()},
                        bounds=scene['bounds'], occupancy_threshold=args.occupancy_threshold,
                        config=str(config), fixed_saved_poses=True, original_timestamps_in_source_files=True)
        manifest['topics']['/tf_static'] = 'tf2_msgs/msg/TFMessage'
        manifest['static_reference'] = dict(parent=args.frame, child=reference.child_frame_id)
        (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(f'Published {len(messages)} saved-result topics in {args.frame}; ROS_DOMAIN_ID={args.domain_id}', flush=True)
        print(f'RViz config and logs: {out}', flush=True)
        if not args.no_rviz:
            from ament_index_python.packages import get_package_prefix
            executable = Path(get_package_prefix('rviz2'))/'lib/rviz2/rviz2'
            log = (out/'rviz.log').open('w')
            child = subprocess.Popen([str(executable), '-d', str(config), '--ros-args', '-r', '__node:=occupancy_rviz'],
                                     stdout=log, stderr=subprocess.STDOUT)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
            if child and child.poll() is not None:
                if child.returncode != 0:
                    raise RuntimeError(f'RViz exited with {child.returncode}; see {out / "rviz.log"}')
                break
            if args.duration and time.monotonic()-started >= args.duration:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        status = 'interrupted'
    except Exception:
        status = 'failed'
        raise
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        if log:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (out/'exit.json').write_text(json.dumps(dict(status=status, elapsed_s=time.monotonic()-started,
                                                   rviz_returncode=child.returncode if child else None), indent=2)+'\n')


def main():
    p = parser()
    args = p.parse_args()
    base = Path(os.environ.get('OCCUPANCY_VIEWER_CALLER_DIR', os.getcwd()))
    for key in ('result', 'initial', 'output'):
        value = getattr(args, key)
        if value is not None:
            setattr(args, key, (base/value).resolve())
    if args.output is None:
        if not os.environ.get('SDF_BA_RUN_ROOT'):
            p.error('Use the data-disk launcher or specify --output for viewer logs/config')
        args.output = Path(os.environ['SDF_BA_RUN_ROOT'])/'rviz'
    if not math.isfinite(args.display_resolution) or args.display_resolution <= 0:
        p.error('--display-resolution must be finite and positive')
    if args.max_points < 0 or not math.isfinite(args.duration) or args.duration < 0 or not 0 <= args.occupancy_threshold <= 1 or not 0 <= args.domain_id <= 232:
        p.error('Invalid point cap, duration, threshold or ROS domain')
    if args.height_range and (not all(math.isfinite(v) for v in args.height_range) or args.height_range[0] >= args.height_range[1]):
        p.error('--height-range requires finite MIN_Z < MAX_Z')
    if not args.frame or args.frame.startswith('/') or any(c.isspace() for c in args.frame):
        p.error('--frame must be a nonempty TF frame without a leading slash or whitespace')
    from rclpy.validate_namespace import validate_namespace
    args.namespace = '/' + args.namespace.strip('/')
    validate_namespace(args.namespace)
    if args.namespace == '/':
        p.error('--namespace must not be empty')
    if not args.no_rviz and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        p.error('No display available; use --no-rviz to publish for a separate RViz instance')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    run(args)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        sys.exit(f'RViz viewer: {exc}')
