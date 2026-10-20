#!/usr/bin/env python3
"""Recheck final 2D binary against the previously validated real ROS replay."""
import json
import os
from pathlib import Path
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

root=Path(__file__).resolve().parents[1]
run=Path(os.environ['SDF_BA_RUN_ROOT'])
reference=Path('/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_cartographer_smoke_20261019_02/pipeline')
out=run/'optimized'
command=[str(root/'build/occupancy_slam_2d'),'--ranges',str(reference/'Range.txt'),'--poses',str(reference/'Pose.txt'),
         '--odom',str(reference/'Odom.txt'),'--timestamps',str(reference/'timestamps.txt'),
         '--config',str(reference/'backend_config.txt'),'--output',str(out)]
with (run/'backend.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
poses=np.loadtxt(out/'poses.txt')
np.testing.assert_allclose(poses,np.loadtxt(reference/'optimized/poses.txt'),atol=1e-12,rtol=0)
report=json.loads((out/'summary.json').read_text())
assert report['iterations']==5 and report['frames']==77
assert np.isfinite(report['final_scan_sse']) and report['final_scan_sse']<report['initial_scan_sse']
values=np.loadtxt(out/'occupancy.txt');normalization=np.loadtxt(out/'normalization.txt')
prob=1/(1+np.exp(-np.clip(values/np.maximum(normalization,1e-9),-30,30)))
assert prob.max()>.65 and prob.min()<.35
meta=yaml.safe_load((out/'map.yaml').read_text())
x,y,_=meta['origin'];resolution=meta['resolution']
initial=np.loadtxt(reference/'Pose.txt')
fig,axes=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
axes[0].imshow(prob,origin='lower',cmap='gray_r',vmin=0,vmax=1,extent=[x,x+values.shape[1]*resolution,y,y+values.shape[0]*resolution])
axes[0].plot(poses[:,0],poses[:,1],color='#2b78c1',linewidth=1,label='C++ optimized')
axes[0].set(title='Revo LDS: optimized occupancy (77 scans)',xlabel='x (m)',ylabel='y (m)');axes[0].legend()
axes[1].plot(initial[:,0],initial[:,1],'.-',label='slam_toolbox initialization',markersize=2)
axes[1].plot(poses[:,0],poses[:,1],'.-',label='C++ optimized',markersize=2)
axes[1].axis('equal');axes[1].set(title='Trajectory comparison (no ground truth)',xlabel='x (m)',ylabel='y (m)');axes[1].legend()
fig.savefig(run/'cartographer_preview.png',dpi=150)
print(json.dumps(report,indent=2));print('Final binary agrees with previous verified trajectory.')
