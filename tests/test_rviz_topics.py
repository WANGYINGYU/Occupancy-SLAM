#!/usr/bin/env python3
"""Exercise native RViz Topic dropdown selection on X11; record DDS changes and real screenshots.

Requires the checked-in 1500x950 RViz templates and standard desktop scaling.
Run through run_on_data.py with DISPLAY/XAUTHORITY set, without concurrent GUI input.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import yaml
from test_rviz import screenshot

ROOT = Path(__file__).resolve().parents[1]


class XInput:
    def __init__(self):
        self.x = ctypes.CDLL('libX11.so.6')
        self.xt = ctypes.CDLL('libXtst.so.6')
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.x.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x.XStringToKeysym.restype = ctypes.c_ulong
        self.x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x.XKeysymToKeycode.restype = ctypes.c_uint
        self.x.XFlush.argtypes = self.x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.xt.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
        self.xt.XTestFakeButtonEvent.argtypes = self.xt.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('X11 display unavailable')

    def key(self, name, down):
        code = self.x.XKeysymToKeycode(self.display, self.x.XStringToKeysym(name.encode()))
        assert code, name
        self.xt.XTestFakeKeyEvent(self.display, code, down, 0)
        self.x.XFlush(self.display)

    def press(self, name):
        self.key(name, True)
        self.key(name, False)

    def chord(self, modifier, key):
        self.key(modifier, True)
        self.press(key)
        self.key(modifier, False)
        time.sleep(.15)

    def click(self, x, y):
        self.xt.XTestFakeMotionEvent(self.display, -1, x, y, 0)
        self.xt.XTestFakeButtonEvent(self.display, 1, True, 0)
        self.xt.XTestFakeButtonEvent(self.display, 1, False, 0)
        self.x.XFlush(self.display)
        time.sleep(.1)

    def topic(self, wid, value, options):
        info = subprocess.check_output(['xwininfo', '-id', wid], text=True)
        left = int(re.search(r'Absolute upper-left X:\s+(-?\d+)', info)[1])
        top = int(re.search(r'Absolute upper-left Y:\s+(-?\d+)', info)[1])
        # Native Topic value row in both checked-in templates (Map / PointCloud2).
        self.click(left+285, top+245)
        self.click(left+285, top+245)
        # Select from the native topic dropdown, without text/IME composition.
        self.chord('Alt_L', 'Down')
        time.sleep(.3)
        self.press('Home')
        for _ in range(options.index(value)):
            self.press('Down')
        self.press('Return')
        self.press('Tab')
        self.click(left+65, top+202)
        time.sleep(.5)
        self.chord('Control_L', 's')

    def close(self):
        self.x.XCloseDisplay(self.display)


def verify(mode, result, out, display_resolution=.05):
    import rclpy
    from rclpy.node import Node
    out.mkdir()
    config = out/'viewer'/f'occupancy_{mode}.rviz'
    command = ['bash', str(ROOT/'tools/view_rviz.sh'), mode, '--result', str(result),
               '--output', str(out/'viewer'), '--domain-id', '190']
    if mode == '2d':
        command += ['--display-resolution', str(display_resolution)]
    node = None
    ui = XInput()
    reports = []
    with (out/'publisher.log').open('w') as log:
        # Keep test key events out of the desktop's Chinese input method.
        env = dict(os.environ, QT_IM_MODULE='compose')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            deadline = time.monotonic()+30
            while not (out/'viewer/manifest.json').exists():
                assert process.poll() is None and time.monotonic() < deadline, 'Publisher failed'
                time.sleep(.2)
            os.environ['ROS_DOMAIN_ID'] = '190'
            rclpy.init(args=[])
            node = Node('native_topic_switch_verification')
            manifest = json.loads((out/'viewer/manifest.json').read_text())
            assert manifest['display'] == 'native_topic_selection'
            assert not any('/comparison/' in t for t in manifest['topics'])
            time.sleep(4)
            capture = screenshot(mode, out/'opened.png', config)
            # Establish RViz's own serialized camera settings before any Topic edits.
            ui.chord('Control_L', 's')
            time.sleep(.5)
            camera = yaml.safe_load(config.read_text())['Visualization Manager']['Views']['Current']
            kind = 'map_display' if mode == '2d' else 'points'
            map_topics = {t for t in manifest['topics'] if t.endswith('/'+kind)}
            for method in ('initial', 'reintegrated', 'optimized'):
                target = f'/occupancy_slam/{method}/{kind}'
                options = sorted(t for t, msg_type in manifest['topics'].items()
                                 if msg_type == manifest['topics'][target])
                ui.topic(capture['window_id'], target, options)
                screenshot(mode, out/(method+'_edited.png'), config)
                deadline = time.monotonic()+20
                while time.monotonic() < deadline:
                    gui = {topic for topic in map_topics if any(e.node_name == 'occupancy_rviz'
                           for e in node.get_subscriptions_info_by_topic(topic))}
                    if gui == {target}:
                        break
                    rclpy.spin_once(node, timeout_sec=.1)
                assert gui == {target}, (target, gui)
                saved = yaml.safe_load(config.read_text())['Visualization Manager']
                display = next(d for d in saved['Displays'] if d['Name'] == ('Map' if mode == '2d' else 'PointCloud2'))
                assert display['Topic']['Value'] == target
                assert display['Topic']['Durability Policy'] == 'Transient Local'
                assert saved['Global Options']['Fixed Frame'] == 'map'
                assert saved['Views']['Current'] == camera, 'Topic switch changed the camera'
                if mode == '3d':
                    assert display['Color Transformer'] == 'RGB8'
                    assert display['Channel Name'] == 'z'
                    assert not display['Autocompute Intensity Bounds']
                    assert abs(display['Min Intensity']-manifest['height_color']['min']) < 1e-5
                    assert abs(display['Max Intensity']-manifest['height_color']['max']) < 1e-5
                time.sleep(2)
                capture = screenshot(mode, out/(method+'.png'), config)
                (out/(method+'.rviz')).write_text(config.read_text())
                reports.append(dict(method=method, topic=target, rviz_subscriptions=sorted(gui),
                                    unchanged_camera=True, **capture))
                print(json.dumps(reports[-1]), flush=True)
            (out/'verification.json').write_text(json.dumps(dict(mode=mode, switches=reports), indent=2)+'\n')
        finally:
            ui.close()
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        assert process.returncode == 0, (out/'publisher.log').read_text()
        exit_info = json.loads((out/'viewer/exit.json').read_text())
        assert exit_info['rviz_returncode'] == 0
        log = (out/'viewer/rviz.log').read_text()
        errors = [line for line in log.splitlines() if '[ERROR]' in line]
        known = mode == '2d' and len(errors) == 1 and 'indexed_8bit_image.vert' in errors[0] and 'active samplers with a different type' in log
        assert (not errors or known) and 'GL_INVALID' not in log and 'RenderingAPIException' not in log, log
        report = dict(mode=mode, switches=reports, exit=exit_info, rviz_errors=errors,
                      known_map_shader_startup_diagnostic=bool(known))
        (out/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
        return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('result2d', 'result3d', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--mode', choices=('2d', '3d', 'both'), default='both')
    p.add_argument('--display-resolution', type=float, default=.05)
    args = p.parse_args()
    args.output.mkdir(exist_ok=False)
    modes = ('2d', '3d') if args.mode == 'both' else (args.mode,)
    reports = [verify(mode, getattr(args, 'result'+mode), args.output/mode, args.display_resolution) for mode in modes]
    (args.output/'verification.json').write_text(json.dumps(reports, indent=2)+'\n')


if __name__ == '__main__':
    main()
