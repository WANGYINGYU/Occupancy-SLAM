#!/usr/bin/env python3
"""Real DDS late-subscriber checks and native RViz window screenshots on saved runs."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from result_io import read_pcd


def screenshot(mode, output, config_path=None):
    from PIL import ImageGrab
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    needle = re.escape(str(config_path)) if config_path else 'occupancy_'+mode+r'\.rviz'
    matches = re.findall(r'(0x[0-9a-f]+) "[^"\n]*'+needle+r'[^"\n]*"', tree)
    if not matches:
        raise RuntimeError('Native RViz window not found')
    wid = int(matches[-1], 16)
    lib = ctypes.CDLL('libX11.so.6')
    lib.XOpenDisplay.restype = ctypes.c_void_p
    lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    for fn in ('XRaiseWindow',):
        getattr(lib, fn).argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    lib.XFlush.argtypes = lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = lib.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot open X display')
    # Request activation through the window manager; direct XRaiseWindow is
    # ignored by some compositors for overlapping managed windows.
    class ClientData(ctypes.Union):
        _fields_ = [('b', ctypes.c_char*20), ('s', ctypes.c_short*10), ('l', ctypes.c_long*5)]
    class ClientEvent(ctypes.Structure):
        _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send_event', ctypes.c_int),
                    ('display', ctypes.c_void_p), ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                    ('format', ctypes.c_int), ('data', ClientData)]
    class Event(ctypes.Union):
        _fields_ = [('client', ClientEvent), ('pad', ctypes.c_long*24)]
    lib.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.XInternAtom.restype = ctypes.c_ulong
    lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    lib.XDefaultRootWindow.restype = ctypes.c_ulong
    lib.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(Event)]
    event = Event()
    event.client.type = 33
    event.client.display = display
    event.client.window = wid
    event.client.message_type = lib.XInternAtom(display, b'_NET_ACTIVE_WINDOW', 0)
    event.client.format = 32
    event.client.data.l[0] = 2
    lib.XSendEvent(display, lib.XDefaultRootWindow(display), 0, (1<<19)|(1<<20), ctypes.byref(event))
    lib.XFlush(display)
    deadline = time.monotonic()+4
    while time.monotonic() < deadline:
        active = subprocess.check_output(['xprop', '-root', '_NET_ACTIVE_WINDOW'], text=True)
        if hex(wid) in active:
            break
        time.sleep(.1)
    else:
        lib.XCloseDisplay(display)
        raise RuntimeError(f'Window manager did not activate {hex(wid)}: {active}')
    time.sleep(1)
    text = subprocess.check_output(['xwininfo', '-id', hex(wid)], text=True)
    def integer(label):
        return int(re.search(re.escape(label)+r':\s*(-?\d+)', text).group(1))
    x, y = integer('Absolute upper-left X'), integer('Absolute upper-left Y')
    w, h = integer('Width'), integer('Height')
    image = ImageGrab.grab(bbox=(x,y,x+w,y+h), xdisplay=os.environ['DISPLAY'])
    image.save(output)
    lib.XCloseDisplay(display)
    return dict(window_id=hex(wid), width=w, height=h, screenshot=str(output))


def verify(mode, args, output, occupancy=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Path as PathMessage
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray
    from tf2_msgs.msg import TFMessage
    result = args.result2d if mode == '2d' else args.result3d
    initial = args.initial2d if mode == '2d' else args.initial3d
    command = ['bash', str(ROOT/'tools/view_rviz.sh'), mode, '--result', str(result), '--initial', str(initial),
               '--output', str(output/'viewer'), '--domain-id', '189']
    if mode == '2d':
        command += ['--display-resolution', str(args.display_resolution)]
    if mode == '3d' and occupancy:
        command.append('--show-occupancy')
    output.mkdir()
    with (output/'publisher.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            manifest_path = output/'viewer/manifest.json'
            start = time.monotonic()
            while not manifest_path.exists():
                if process.poll() is not None or time.monotonic()-start > 30:
                    raise RuntimeError(f'Publisher failed: {output / "publisher.log"}')
                time.sleep(.2)
            # Publishers send once. Create subscribers AFTER messages were sent.
            time.sleep(3)
            manifest = json.loads(manifest_path.read_text())
            os.environ['ROS_DOMAIN_ID'] = '189'
            rclpy.init(args=[])
            node = Node('occupancy_rviz_verification')
            received = {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            types = dict(OccupancyGrid=OccupancyGrid, Path=PathMessage, PointCloud2=PointCloud2, MarkerArray=MarkerArray, TFMessage=TFMessage)
            subscriptions = [node.create_subscription(types[kind.rsplit('/', 1)[-1]], topic,
                             lambda message, topic=topic: received.__setitem__(topic, message), qos)
                             for topic, kind in manifest['topics'].items()]
            deadline = time.monotonic()+25
            while len(received) < len(subscriptions) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert len(received) == len(subscriptions), set(manifest['topics'])-set(received)
            for topic, msg in received.items():
                if hasattr(msg, 'header'):
                    assert msg.header.frame_id == 'map'
                if isinstance(msg, PathMessage):
                    name = topic.split('/')[-2]
                    expected = np.loadtxt((initial if name == 'initial' else result)/'trajectory.tum', ndmin=2) if mode == '3d' else np.loadtxt(result/('Pose.txt' if name == 'initial' else 'optimized/poses.txt'), ndmin=2)
                    actual = np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
                    target = expected[:,1:4] if mode == '3d' else np.column_stack((expected[:,:2],np.zeros(len(expected))))
                    np.testing.assert_allclose(actual,target,atol=1e-12,rtol=0)
                    assert all(abs(sum(getattr(p.pose.orientation,q)**2 for q in ('x','y','z','w'))-1)<1e-8 for p in msg.poses)
                elif isinstance(msg, PointCloud2):
                    relative = topic.removeprefix('/occupancy_slam/')
                    assert msg.width == manifest['clouds'][relative]['published_points']
                    assert len(msg.data) == msg.width*msg.point_step == msg.row_step
                    xyz = np.frombuffer(msg.data,dtype='<f4').reshape(msg.width,-1)
                    assert np.isfinite(xyz).all()
                    if mode == '3d':
                        name, kind = relative.split('/')
                        folder = initial if name=='initial' else result/'reintegrated' if name=='reintegrated' else result
                        path = folder/('optimized_points.pcd' if kind=='points' else 'occupancy_vertices.pcd')
                        source, intensity = read_pcd(path)
                        if kind == 'occupancy':
                            mask = intensity >= .6
                            source = np.column_stack((source[mask],intensity[mask]))
                        np.testing.assert_array_equal(xyz[:, :source.shape[1]],source.astype('<f4'))
                        rgb_field = next(f for f in msg.fields if f.name == 'rgb')
                        assert rgb_field.datatype == 6 and rgb_field.offset == msg.point_step-4
                        colors = np.frombuffer(msg.data,dtype='<u4').reshape(msg.width,-1)[:,-1]
                        assert np.all(colors <= 0xFFFFFF) and len(np.unique(colors)) > 10
                        style = manifest['height_color']
                        assert style['shared_across_methods'] and style['channel'] == 'z'
                elif isinstance(msg, OccupancyGrid):
                    name = topic.split('/')[-2]
                    folder = initial if name == 'initial' else result/'optimized/reintegrated' if name=='reintegrated' else result/'optimized'
                    raw = np.loadtxt(folder/'occupancy.txt')
                    count = np.loadtxt(folder/'normalization.txt')
                    actual = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
                    if topic.endswith('/map_display'):
                        display = manifest['display_maps'][name]
                        with np.load(Path(display['files'])/'grid.npz') as saved:
                            np.testing.assert_array_equal(actual, saved['values'])
                            assert set(np.unique(actual)) == {-1, 0, 100}
                            if name != 'optimized':
                                probability = 1/(1+np.exp(-np.clip(saved['accumulated_log_odds'], -30, 30)))
                                observed = saved['integration_counts'] > 0
                                expected = np.full(actual.shape, -1)
                                expected[observed & (probability <= .35)] = 0
                                expected[observed & (probability >= .65)] = 100
                                np.testing.assert_array_equal(actual, expected)
                                assert abs(msg.info.resolution-args.display_resolution) < 1e-8
                            else:
                                probability = 1/(1+np.exp(-np.clip(raw, -30, 30)))
                                assert actual.shape == raw.shape
                                assert np.all(probability[actual == 0] <= .35)
                                assert np.all(probability[actual == 100] >= .65)
                        assert abs(msg.info.resolution-display['resolution']) < 1e-8
                        np.testing.assert_allclose([msg.info.origin.position.x,msg.info.origin.position.y],
                                                   display['origin'][:2], atol=1e-12)
                    else:
                        expected = np.rint(100/(1+np.exp(-np.clip(raw/np.maximum(count,1e-9),-30,30)))).astype(np.int8)
                        assert (msg.info.height,msg.info.width) == expected.shape
                        np.testing.assert_array_equal(actual, expected)
                elif isinstance(msg, TFMessage):
                    assert len(msg.transforms) == 1
                    t = msg.transforms[0]
                    assert t.header.frame_id == 'map'
                    assert t.child_frame_id == 'occupancy_slam_display_origin'
                    assert t.transform.rotation.w == 1.
                elif isinstance(msg, MarkerArray):
                    assert len(msg.markers)==2*len(manifest['trajectories'])
                    assert all(m.header.frame_id=='map' for m in msg.markers)
            # Require RViz itself to be a discovered subscriber, not only this test.
            required_topic = '/occupancy_slam/'+manifest['selected_method']+'/'+('map_display' if mode=='2d' else 'occupancy' if occupancy else 'points')
            gui_topics = []
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                gui_topics = [topic for topic in manifest['topics'] if any(
                    'rviz' in e.node_name and e.node_name != 'occupancy_rviz_verification'
                    for e in node.get_subscriptions_info_by_topic(topic))]
                if required_topic in gui_topics:
                    break
                rclpy.spin_once(node, timeout_sec=.1)
            assert required_topic in gui_topics, (gui_topics, node.get_node_names())
            time.sleep(3)
            capture = screenshot(mode, output/'rviz.png', output/'viewer'/f'occupancy_{mode}.rviz')
            answer = dict(mode=mode, occupancy_view=occupancy, all_topics_received=len(received),
                          late_subscriber_delay_s=3, rviz_subscriptions=gui_topics,
                          clouds=manifest['clouds'], trajectories=manifest['trajectories'], maps=manifest['maps'], display_maps=manifest['display_maps'], **capture)
            node.destroy_node()
            rclpy.shutdown()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait()
        assert process.returncode == 0, (output/'publisher.log').read_text()
        rviz_log = (output/'viewer/rviz.log').read_text()
        # Jazzy MapDisplay can emit one startup shader diagnostic (ros2/rviz#463).
        # Retain it explicitly; screenshot review is required to establish rendering.
        errors = [line for line in rviz_log.splitlines() if '[ERROR]' in line]
        known_map_startup = (mode == '2d' and len(errors) == 1 and
                             'indexed_8bit_image.vert' in errors[0] and
                             'active samplers with a different type' in rviz_log)
        answer['rviz_errors'] = errors
        answer['known_map_shader_startup_diagnostic'] = known_map_startup
        assert (not errors or known_map_startup) and 'GL_INVALID' not in rviz_log and 'RenderingAPIException' not in rviz_log, rviz_log
        answer['exit'] = json.loads((output/'viewer/exit.json').read_text())
        assert answer['exit']['rviz_returncode'] == 0, answer['exit']
        (output/'verification.json').write_text(json.dumps(answer,indent=2)+'\n')
        return answer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('result2d','initial2d','result3d','initial3d','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--display-resolution',type=float,default=.05)
    p.add_argument('--mode',choices=('2d','3d','both'),default='both')
    args = p.parse_args()
    args.output.mkdir(exist_ok=False)
    results=[]
    for mode,occ in [('2d',False),('3d',False),('3d',True)]:
        if args.mode != 'both' and mode != args.mode: continue
        name=mode+('_occupancy' if occ else '')
        result=verify(mode,args,args.output/name,occ)
        results.append(result)
        print(json.dumps(result),flush=True)
    (args.output/'verification.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
