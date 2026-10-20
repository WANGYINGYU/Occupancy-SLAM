#!/usr/bin/env python3
"""Download a TXT example from the standalone release into a data-disk run."""
import argparse
import json
import os
from pathlib import Path
import shutil
from urllib.request import urlopen

# Versioned source of the original research datasets (also on C_plus_plus).
SOURCE_REVISION = '705b2178f2d6f863a85d1705a6631389b9a8de9a'
SOURCE_URL = 'https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/'
FILES = {
    'Intel': ('Range.txt', 'Pose.txt', 'Odom.txt', 'README.md'),
    'Museum_b0': ('Range.txt', 'Pose.txt', 'README.md'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=FILES, default='Intel')
    args = parser.parse_args()
    if 'SDF_BA_RUN_ROOT' not in os.environ:
        parser.error('Run through tools/run_on_data.py or tools/docker.sh exec')
    output = Path(os.environ['SDF_BA_RUN_ROOT']) / args.dataset
    output.mkdir(exist_ok=False)
    sources = {}
    for name in FILES[args.dataset]:
        url = f'{SOURCE_URL}{SOURCE_REVISION}/Data/{args.dataset}/{name}'
        # Publish each file only after the response has been fully received.
        partial = output / (name + '.part')
        with urlopen(url, timeout=60) as response, partial.open('xb') as stream:
            shutil.copyfileobj(response, stream)
        partial.rename(output / name)
        sources[name] = url
    (output/'source.json').write_text(json.dumps(sources, indent=2) + '\n')
    print(output, flush=True)


if __name__ == '__main__':
    main()
