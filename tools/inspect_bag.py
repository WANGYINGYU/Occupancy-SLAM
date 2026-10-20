#!/usr/bin/env python3
"""List bag topics before choosing an input topic; supports ROS1 without ROS1 installed."""
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('bag',type=Path)
a=p.parse_args()
with AnyReader([a.bag.expanduser().resolve()]) as r:
    for c in r.connections:
        print(f'{c.topic}\t{c.msgtype}\t{c.msgcount}')
