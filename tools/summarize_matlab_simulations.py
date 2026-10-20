#!/usr/bin/env python3
"""Aggregate complete MATLAB simulation runs and render reproducible figures."""
import argparse
import csv
import json
import os
from pathlib import Path
import time
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np
import yaml


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save(fig, out, name):
    fig.savefig(out/(name+'.png'), dpi=180, bbox_inches='tight')
    fig.savefig(out/(name+'.pdf'), bbox_inches='tight')
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('results', 'maps', 'converted'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--wait', action='store_true')
    a = p.parse_args()
    out = Path(os.environ['OCCUPANCY_RUN_DIR'])
    shutil.copy2(__file__,out/'report_source.py')
    inventory = json.loads((a.converted/'inventory.json').read_text())
    expected = {(s['scene'], c['name']) for s in inventory for c in s['cases']}
    while True:
        try:
            runs = json.loads((a.results/'metrics.json').read_text())
            maps = json.loads((a.maps/'metrics.json').read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            runs, maps = [], []
        if len(runs) == len(expected) and any(r['status'] != 'completed' for r in runs):
            raise RuntimeError('Failed runs must be reported explicitly before aggregation')
        if len(runs) == len(expected) and len(maps) == 6*len(expected):
            break
        if not a.wait:
            raise ValueError(f'Incomplete results: {len(runs)} runs, {len(maps)} map rows')
        time.sleep(5)
    assert {(r['scene'], r['case']) for r in runs} == expected
    assert len({(r['scene'], r['case'], r['method'], r['probability_threshold']) for r in maps}) == 6*len(expected)
    lookup = {(r['scene'], r['case'], r['method'], r['probability_threshold']): r for r in maps}
    # The main table has five supplied repeats per scene and initialization type.
    # The separately named sixth noisy-odometry input remains visible as an extra.
    standard = [r for r in runs if r['case'] != 'odom_24']
    extra = [r for r in runs if r['case'] == 'odom_24']
    scenes = [s['scene'] for s in inventory]
    kinds = ['noisy_odometry', 'cartographer']
    grouped, grouped_maps = [], []
    for scene in scenes:
        for kind in kinds:
            subset = [r for r in standard if r['scene'] == scene and r['kind'] == kind]
            assert len(subset) == 5
            row = dict(scene=scene, kind=kind, repeats=len(subset))
            for key in ('translation_rmse_m', 'heading_rmse_deg', 'rpe1_translation_rmse_m', 'rpe1_heading_rmse_deg'):
                for method in ('initial', 'joint'):
                    values = [r[method+'_'+key] for r in subset]
                    row[method+'_'+key+'_mean'] = float(np.mean(values))
                    row[method+'_'+key+'_std'] = float(np.std(values, ddof=1))
                row[key+'_improved_count'] = sum(r['joint_'+key] < r['initial_'+key] for r in subset)
            row['wall_seconds_mean'] = float(np.mean([r['wall_seconds'] for r in subset]))
            row['peak_rss_gib_max_sampled'] = max(r['peak_rss_kib_sampled'] for r in subset)/1024**2
            grouped.append(row)
            for threshold in (.5, .65):
                row = dict(scene=scene, kind=kind, repeats=5, probability_threshold=threshold)
                for method in ('initial', 'joint', 'reintegrated'):
                    selected = [lookup[scene, r['case'], method, threshold] for r in subset]
                    for metric in ('symmetric_capped_mae_m', 'forward_mae_m', 'backward_mae_m', 'f1_010', 'f1_020'):
                        values=[r[metric] for r in selected]
                        row[method+'_'+metric+'_defined_count']=sum(v is not None for v in values)
                        # A missing contour has no uncapped distance; do not silently drop it.
                        row[method+'_'+metric+'_mean']=float(np.mean(values)) if all(v is not None for v in values) else None
                    row[method+'_empty_contour_count']=sum(r['predicted_points']==0 for r in selected)
                grouped_maps.append(row)
    write_csv(out/'trajectory_grouped.csv', grouped)
    write_csv(out/'map_grouped.csv', grouped_maps)
    write_csv(out/'all_trajectory_metrics.csv', runs)
    write_csv(out/'all_map_metrics.csv', maps)
    write_csv(out/'extra_initialization.csv', extra)
    summary = dict(cases=len(runs), scenes=len(scenes), standard_cases=len(standard), extra_cases=len(extra),
                   translation_improved=sum(r['translation_improved'] for r in runs),
                   heading_improved=sum(r['heading_improved'] for r in runs),
                   rpe1_translation_improved=sum(r['joint_rpe1_translation_rmse_m'] < r['initial_rpe1_translation_rmse_m'] for r in runs),
                   rpe1_heading_improved=sum(r['joint_rpe1_heading_rmse_deg'] < r['initial_rpe1_heading_rmse_deg'] for r in runs),
                   objective_increased=[r['scene']+'/'+r['case'] for r in runs if r['final_cost'] > r['initial_cost']],
                   stops={k: sum(r['stop_reason'] == k for r in runs) for k in sorted({r['stop_reason'] for r in runs})},
                   wall_seconds_sum=sum(r['wall_seconds'] for r in runs),
                   peak_rss_gib_max_sampled=max(r['peak_rss_kib_sampled'] for r in runs)/1024**2,
                   peak_swap_kib_max_sampled=max(r['peak_swap_kib_sampled'] for r in runs),
                   per_run_wall_seconds_range=[min(r['wall_seconds'] for r in runs), max(r['wall_seconds'] for r in runs)],
                   main_table='Mean of five per-run RMSEs; not pooled frame RMSE. Noise_24 is separate.',
                   empty_contours=[dict(scene=r['scene'],case=r['case'],method=r['method'],threshold=r['probability_threshold']) for r in maps if not r['predicted_points']],
                   map_comparisons={})
    for threshold in (.5, .65):
        counts = {}
        for metric in ('symmetric_capped_mae_m', 'f1_010', 'f1_020'):
            for method, baseline in [('joint', 'initial'), ('reintegrated', 'initial'), ('joint', 'reintegrated')]:
                direction = 1 if metric.startswith('f1') else -1
                counts[method+'_better_than_'+baseline+'_'+metric] = sum(
                    direction*(lookup[s, c, method, threshold][metric]-lookup[s, c, baseline, threshold][metric]) > 0 for s, c in expected)
        summary['map_comparisons'][str(threshold)] = counts
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (out/'sources.json').write_text(json.dumps(dict(results=str(a.results), maps=str(a.maps), converted=str(a.converted),
        representative_case='carto_1 in every scene, fixed index; not best result'), indent=2)+'\n')
    tex = ['% Means of five per-run RMSEs; fixed shared first-pose SE2, no scale.',
           r'\begin{tabular}{llrrrr}', r'\hline',
           r'Scene & Initializer & $t_0$ (m) & $t_1$ (m) & $\theta_0$ ($^\circ$) & $\theta_1$ ($^\circ$) \\', r'\hline']
    for r in grouped:
        label = 'Odometry' if r['kind'] == 'noisy_odometry' else 'Cartographer'
        vals = [r[m+'_'+k+'_mean'] for k in ('translation_rmse_m', 'heading_rmse_deg') for m in ('initial', 'joint')]
        tex.append(r['scene']+' & '+label+' & '+' & '.join(f'{v:.4f}' for v in vals)+r' \\')
    tex += [r'\hline', r'\end{tabular}']
    (out/'trajectory_table.tex').write_text('\n'.join(tex)+'\n')
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), constrained_layout=True)
    for i, scene in enumerate(scenes):
        subset = sorted([r for r in runs if r['scene'] == scene], key=lambda r: (r['kind'] == 'cartographer', int(r['case'].split('_')[-1])))
        x = np.arange(len(subset))
        for j, (key, unit) in enumerate([('translation_rmse_m', 'Position RMSE (m)'), ('heading_rmse_deg', 'Heading RMSE (deg)')]):
            ax = axes[i, j]
            ax.bar(x-.18, [r['initial_'+key] for r in subset], .36, label='Initial', color='#7f8c8d')
            ax.bar(x+.18, [r['joint_'+key] for r in subset], .36, label='Optimized', color='#2077b4')
            ax.set_xticks(x, [r['case'].replace('odom_', 'O').replace('carto_', 'C') for r in subset])
            ax.set_title(scene); ax.set_ylabel(unit); ax.grid(axis='y', alpha=.2)
            if i == 0 and j == 0: ax.legend()
    fig.suptitle('All 41 supplied initializations: fixed shared SE(2), final accepted state')
    save(fig, out, 'pose_errors_all')
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for ax, scene in zip(axes.flat, scenes):
        case = a.results/scene/'carto_1'
        indices = np.loadtxt(case/'gt_indices.txt', dtype=int)
        gt = np.loadtxt(a.converted/scene/'Pose_GT.txt')[indices]
        ref = np.load(a.maps/(scene+'_reference.npz'))['points']
        ax.scatter(*ref[::8].T, s=.2, c='0.75', rasterized=True)
        for method, color in [('initial', '#d77b22'), ('joint', '#2077b4')]:
            poses = np.loadtxt(case/'evaluation'/(method+'_aligned_poses.txt'))
            ax.plot(*poses[:, :2].T, color=color, lw=1, label=method)
        ax.plot(*gt[:, :2].T, color='black', lw=.8, ls='--', label='GT')
        ax.set_aspect('equal'); ax.set_title(scene+' / carto_1'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    axes[0, 0].legend()
    save(fig, out, 'trajectories_fixed_carto1')
    fig, axes = plt.subplots(4, 3, figsize=(15, 18), constrained_layout=True)
    for i, scene in enumerate(scenes):
        case = a.results/scene/'carto_1'
        ref = np.load(a.maps/(scene+'_reference.npz'))['points']
        for j, method in enumerate(('initial', 'joint', 'reintegrated')):
            ax = axes[i, j]; folder = case/'optimized'/method
            m = np.loadtxt(folder/'occupancy.txt'); n = np.loadtxt(folder/'normalization.txt')
            meta = yaml.safe_load((folder/'map.yaml').read_text()); h=meta['resolution']; x,y=meta['origin'][:2]
            probability=1/(1+np.exp(-np.clip(m/np.maximum(n,1e-9),-30,30)))
            probability=np.ma.array(probability, mask=n<=1e-9)
            cmap=plt.get_cmap('gray_r').copy();cmap.set_bad('#b3b3b3')
            alignment=json.loads((case/'evaluation/metrics.json').read_text())['shared_alignment']
            transform=Affine2D().rotate(alignment['yaw_rad']).translate(*alignment['translation_m'])+ax.transData
            ax.imshow(probability,origin='lower',cmap=cmap,vmin=0,vmax=1,interpolation='nearest',
                      extent=[x-h/2,x+(m.shape[1]-.5)*h,y-h/2,y+(m.shape[0]-.5)*h],transform=transform)
            ax.scatter(*ref[::12].T,s=.35,c='#d62728',alpha=.6,rasterized=True)
            lower=ref.min(axis=0)-1; upper=ref.max(axis=0)+1
            ax.set_xlim(lower[0],upper[0]);ax.set_ylim(lower[1],upper[1]);ax.set_aspect('equal')
            ax.set_title(scene+' / '+method);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    fig.suptitle('Native occupancy fields at their recorded resolution, fixed carto_1; red = independent true geometry\nContinuous probability shown; quantitative contours evaluated at p = 0.50 and 0.65')
    save(fig,out,'native_maps_fixed_carto1')
    fig,axes=plt.subplots(2,1,figsize=(13,8),constrained_layout=True)
    for ax,threshold in zip(axes,[.5,.65]):
        rows=[r for r in grouped_maps if r['probability_threshold']==threshold];x=np.arange(len(rows))
        for j,(method,color) in enumerate([('initial','#7f8c8d'),('joint','#2077b4'),('reintegrated','#d77b22')]):
            ax.bar(x+(j-1)*.25,[r[method+'_symmetric_capped_mae_m_mean'] for r in rows],.25,color=color,label=method)
        ax.set_xticks(x,[r['scene']+'\n'+('Odom' if r['kind']=='noisy_odometry' else 'Carto') for r in rows])
        ax.set_ylabel('Capped symmetric distance (m)');ax.set_title(f'p = {threshold:.2f}; lower is better');ax.grid(axis='y',alpha=.2)
    axes[0].legend(ncol=3)
    save(fig,out,'map_errors_grouped')
    print(json.dumps(summary,indent=2),flush=True)


if __name__ == '__main__':
    main()
