#!/usr/bin/env python3
"""Evaluate all three native maps of a completed evaluate_room_simulation.py run."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from evaluate_2d_map import evaluate_room_maps


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    args=parser.parse_args()
    run=Path(os.environ['SDF_BA_RUN_ROOT']); source=args.source.resolve()
    gt=np.loadtxt(source/'ground_truth.txt'); reports={}; flat=[]
    for case in ('laser_tf','laser_odom','cloud_tf','cloud_scan_match'):
        pose=json.loads((source/(case+'_evaluation')/'metrics.json').read_text())
        report,arrays=evaluate_room_maps(source/case/'optimized',gt[pose['matched_gt_rows'],1:],pose['shared_alignment'])
        reports[case]=report
        np.savez_compressed(run/(case+'_ray_ranges.npz'),**arrays)
        for method,thresholds in report['methods'].items():
            for threshold,m in thresholds.items():
                flat.append(dict(case=case,method=method,probability_threshold=threshold,
                    rays=report['rays'],coverage=m['coverage'],penalized_mae_m=m['penalized_mae_m'],
                    hit_mae_m=m['hit_error_m']['mean'] if m['hit_error_m'] else None))
        print(case,'map evaluation complete',flush=True)
    (run/'source.json').write_text(json.dumps(dict(source=str(source)),indent=2))
    (run/'metrics.json').write_text(json.dumps(reports,indent=2))
    with (run/'metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)


if __name__=='__main__': main()
