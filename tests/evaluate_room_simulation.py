#!/usr/bin/env python3
"""Quantify the existing ONE rectangular-room fixture through four input adapters.

This is not the paper's Simu 1/2/3 dataset. GT is used only by the evaluator.
"""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from make_fixtures import make_bags
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from evaluate_2d import evaluate


def main():
    run = Path(os.environ['SDF_BA_RUN_ROOT'])
    (run/'bags').mkdir()
    topics = dict(laser='/robot/front/ranges', cloud='/lidar/filtered_cloud',
                  odom='/robot/wheel/odometry', tf='/robot/transforms', tf_static='/robot/static_transforms')
    laser, cloud = make_bags(run/'bags', topics, decoy=True)
    # Independent scalar expression for the simulator's world-to-laser truth.
    i = np.arange(24); yaw = .004*i
    gt = np.column_stack([1000+i, .03*i+.4*np.cos(yaw)+.1*np.sin(yaw),
                          .05*np.sin(i/10)+.4*np.sin(yaw)-.1*np.cos(yaw), yaw+.3])
    np.savetxt(run/'ground_truth.txt', gt, fmt='%.17g')
    (run/'simulation.json').write_text(json.dumps(dict(
        scope='one existing rectangular-room fixture, four adapters; NOT paper Simu 1/2/3',
        frames=24, world_bounds_m=[-4,5,-3,4], sensor_extrinsic_xyyaw=[.4,-.1,.3],
        source=str(ROOT/'tests/make_fixtures.py'), noise='no injected range or prior noise',
        ground_truth='ground_truth.txt', policy='legacy', iterations=10,
        optimization_resolution_m=.3, display_resolution_m=.05), indent=2))
    commands=[]; rows=[]
    cases=[('laser_tf',laser,'tf',topics['laser']),
           ('laser_odom',laser,'odom',topics['laser']),
           ('cloud_tf',cloud,'tf',topics['cloud']),
           ('cloud_scan_match',cloud,'scan-match',topics['cloud'])]
    for j,(name,bag,source,topic) in enumerate(cases):
        out=run/name
        command=[sys.executable,str(ROOT/'tools/rosbag_pipeline.py'),'--bag',str(bag),
            '--output',str(out),'--scan-topic',topic,'--odom-source',source,
            '--odom-topic',topics['odom'],'--tf-topic',topics['tf'],'--tf-static-topic',topics['tf_static'],
            '--odom-frame','odom','--iterations','10','--resolution','.3','--beam-stride','2',
            '--bins','360','--max-range','15','--tf-max-gap','1.1','--domain-id',str(184+j)]
        commands.append(command)
        with (run/(name+'.log')).open('w') as log:
            child=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        if child.returncode:
            rows.append(dict(case=name,status='failed',exit_code=child.returncode))
            print(name,'FAILED',flush=True)
            continue
        metrics=evaluate(out,run/'ground_truth.txt',run/(name+'_evaluation'))
        backend=json.loads((out/'optimized/summary.json').read_text())
        entry=dict(case=name,status='completed',input_frames=24,matched_frames=metrics['matched_frames'],
                   iterations=backend['iterations'],stop_reason=backend['stop_reason'],
                   initial_cost=backend['initial_total_cost'],final_cost=backend['final_total_cost'])
        for method in ('initial','joint','reintegrated'):
            m=metrics['methods'][method]
            entry[method+'_translation_rmse_m']=m['translation_m']['rmse']
            entry[method+'_heading_rmse_deg']=m['heading_deg']['rmse']
            entry[method+'_rpe1_translation_rmse_m']=m['rpe_1_frame']['translation_m']['rmse']
            entry[method+'_rpe1_heading_rmse_deg']=m['rpe_1_frame']['heading_deg']['rmse']
        rows.append(entry)
        print(name,json.dumps(metrics['change']),flush=True)
    (run/'commands.json').write_text(json.dumps(commands,indent=2))
    (run/'metrics.json').write_text(json.dumps(rows,indent=2))
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with (run/'metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    good=[r for r in rows if r['status']=='completed']; x=np.arange(len(good))
    for ax,metric,title in zip(axes,['translation_rmse_m','heading_rmse_deg'],['Position RMSE (m)','Heading RMSE (degrees)']):
        for offset,method,label in [(-.18,'initial','Before C++'),(.18,'joint','After C++')]:
            ax.bar(x+offset,[r[method+'_'+metric] for r in good],width=.36,label=label)
        ax.set_xticks(x,[r['case'] for r in good],rotation=15)
        ax.set_ylabel(title);ax.legend();ax.grid(axis='y',alpha=.2)
    fig.suptitle('One rectangular room, four input paths | shared first-pose gauge | no scale fitting')
    fig.savefig(run/'pose_errors.png',dpi=170)
    fig.savefig(run/'pose_errors.pdf')
    plt.close(fig)
    if len(good)!=len(cases): raise RuntimeError('Incomplete fixture runs; see metrics.json')


if __name__=='__main__': main()
