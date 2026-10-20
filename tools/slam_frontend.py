"""ROS2 slam_toolbox adapter. Returns timestamp-associated causal poses."""
import os, math, time, signal, subprocess, json
from pathlib import Path
import numpy as np
import yaml
from scan_io import stamp_ns
from project_paths import ROOT
def run_slam(scans, priors, a, out):
    # UDP transport avoids Fast DDS runtime files in /dev/shm.
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(ROOT/'config/fastdds_udp.xml')
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(ROOT/'config/fastdds_udp.xml')
    os.environ['ROS_DOMAIN_ID'] = str(a.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from slam_toolbox.srv import SerializePoseGraph

    config_path = a.slam_config or Path(get_package_share_directory('slam_toolbox'))/'config/mapper_params_online_sync.yaml'
    config = yaml.safe_load(config_path.read_text())
    params = config.get('slam_toolbox', config.get('/**', {})).get('ros__parameters')
    if not isinstance(params, dict):
        raise ValueError('slam config must contain slam_toolbox.ros__parameters')
    params.update(use_sim_time=True, odom_frame='occupancy_odom', map_frame='occupancy_map',
                  base_frame='occupancy_laser', scan_topic='/occupancy_scan',
                  use_map_saver=False, enable_interactive_mode=False, throttle_scans=1,
                  minimum_time_interval=0.0, minimum_travel_distance=0.0, minimum_travel_heading=0.0,
                  check_min_dist_and_heading_precisely=True, transform_timeout=0.5,
                  tf_buffer_duration=120., transform_publish_period=0.0,
                  resolution=a.frontend_resolution if a.frontend_resolution is not None else params.get('resolution',.05), min_laser_range=scans[0].range_min,
                  max_laser_range=scans[0].range_max, mode='mapping')
    effective = out/'slam_toolbox.yaml'
    effective.write_text(yaml.safe_dump({'slam_toolbox': {'ros__parameters': params}}))
    executable = Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/sync_slam_toolbox_node'
    rclpy.init()
    node = Node('occupancy_bag_driver')
    reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    scan_pub = node.create_publisher(LaserScan, '/occupancy_scan', reliable)
    tf_pub = node.create_publisher(TFMessage, '/tf', reliable)
    clock_pub = node.create_publisher(Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    poses = {}
    def receive(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses[stamp_ns(msg.header.stamp)] = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
    subscription = node.create_subscription(PoseWithCovarianceStamped, '/pose', receive, reliable)
    log = (out/'slam_toolbox.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(effective)], stdout=log, stderr=subprocess.STDOUT)
    accepted, rejected = [], []
    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError(f'slam_toolbox exited ({process.returncode}); see {out / "slam_toolbox.log"}')
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(node, timeout_sec=0.02)
        return True
    try:
        client = node.create_client(ChangeState, '/slam_toolbox/change_state')
        if not spin_until(client.service_is_ready, a.startup_timeout):
            raise RuntimeError('slam_toolbox lifecycle service timeout')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            if not spin_until(future.done, a.startup_timeout) or not future.result().success:
                raise RuntimeError(f'slam_toolbox lifecycle transition failed: {transition}')
        if not spin_until(lambda: scan_pub.get_subscription_count() > 0 and tf_pub.get_subscription_count() > 0, a.startup_timeout):
            raise RuntimeError('slam_toolbox scan/TF subscriber discovery timeout')
        for index, (scan, prior) in enumerate(zip(scans, priors)):
            msg = LaserScan()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(scan.time, 1_000_000_000)
            msg.header.frame_id = 'occupancy_laser'
            msg.angle_min, msg.angle_increment = scan.angle_min, scan.angle_increment
            msg.angle_max = scan.angle_min + (len(scan.ranges)-1)*scan.angle_increment
            msg.range_min, msg.range_max = scan.range_min, scan.range_max
            msg.ranges = scan.ranges.astype(np.float32).tolist()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = msg.header.stamp, 'occupancy_odom', 'occupancy_laser'
            tf.transform.translation.x, tf.transform.translation.y = float(prior[0]), float(prior[1])
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(prior[2]/2), math.cos(prior[2]/2)
            clock = Clock()
            clock.clock = msg.header.stamp
            clock_pub.publish(clock)
            tf_pub.publish(TFMessage(transforms=[tf]))
            # Give TF a chance to arrive; message filter also queues the scan until it does.
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.01)
            scan_pub.publish(msg)
            if spin_until(lambda: scan.time in poses, a.scan_timeout):
                accepted.append(index)
            else:
                rejected.append({'index': index, 'time_ns': scan.time, 'reason': 'no_matching_slam_pose_before_timeout'})
            if index % 20 == 0 or index == len(scans)-1:
                print(f'slam_toolbox: {index+1}/{len(scans)} sent, {len(accepted)} accepted, {len(rejected)} skipped', flush=True)
        # Save the final graph as well: per-scan /pose messages are causal estimates;
        # later loop closures do not retroactively rewrite earlier /pose messages.
        serializer = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not spin_until(serializer.service_is_ready, 5):
            raise RuntimeError('Missing serialize_map service')
        req = SerializePoseGraph.Request()
        req.filename = str(out/'slam_graph')
        future = serializer.call_async(req)
        if not spin_until(future.done, 30) or future.result().result != 0:
            raise RuntimeError('Could not serialize slam_toolbox graph')
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        (out/'scan_acceptance.json').write_text(json.dumps({'accepted_indices': accepted, 'rejected': rejected}, indent=2))
    if len(accepted) < 2 or len(accepted)/len(scans) < a.min_accepted_fraction:
        raise RuntimeError(f'Only {len(accepted)}/{len(scans)} scans received exact timestamp poses; see scan_acceptance.json')
    return accepted, np.array([poses[scans[i].time] for i in accepted])
