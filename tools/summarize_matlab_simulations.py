#!/usr/bin/env python3
"""Aggregate complete MATLAB simulation runs and render reproducible figures."""
import argparse
import csv
import html
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


def report(out, grouped, grouped_maps, summary, runs):
    from backend_config import read_parameters
    sources=json.loads((out/'sources.json').read_text());result_root=Path(sources['results'])
    first=runs[0];params=read_parameters(result_root/first['scene']/first['case']/'backend_config.txt')
    protocol=json.loads((result_root/'protocol.json').read_text())
    def table(headers, rows):
        return '<table><tr>'+''.join('<th>'+html.escape(str(h))+'</th>' for h in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row)+'</tr>' for row in rows)+'</table>'
    text='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Occupancy-SLAM simulation 精度复核</title><style>body{max-width:1150px;margin:36px auto;padding:0 22px;font:16px/1.75 system-ui;color:#243039}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border-bottom:1px solid #ddd;padding:8px;text-align:right}th:first-child,td:first-child{text-align:left}h1{font-size:28px}h2{font-size:23px;margin-top:35px}img{max-width:100%}.note{padding:16px;background:#edf3f6;border-left:4px solid #547486}a{color:#135d94}</style><h1>四场景 simulation：完整定量复测</h1>'
    text+=f'<p class="note">实际完成 {len(runs)}/{len(runs)} 组。位置 RMSE 改善 {summary["translation_improved"]}/{len(runs)}，航向 RMSE 改善 {summary["heading_improved"]}/{len(runs)}。停止原因：{html.escape(str(summary["stops"]))}。N 的 pose 导数始终省略，每次接受更新后从零重算并平滑 N。未按 GT 选择最优中间步。</p>'
    if params.get('ModeMulti') == 'true':
        text+='<p>本表使用恢复后的 C++ 粗到细路径；它与当前保存的 MATLAB 单分辨率配置分别测试，不宣称两阶段与 MATLAB 逐步相同。粗阶段与精细选区阶段分别记录代价，表中的 cost 变化只指最后一个优化阶段。</p>'
    if sources.get('matlab_reference'):
        from scipy.io import loadmat
        reference=Path(sources['matlab_reference']);identity=json.loads((reference/'completion.json').read_text());case=result_root/identity['scene']/identity['case'];diffs=[]
        for pose_file in sorted(reference.glob('Pose_*.mat'),key=lambda p:int(p.stem.split('_')[1])):
            i=int(pose_file.stem.split('_')[1]);cpp=case/'optimized/steps/fine'/str(i)/'poses.txt'
            if not cpp.exists():continue
            m=loadmat(pose_file,variable_names=['Pose'])['Pose'];c=np.loadtxt(cpp);difference=c-m;difference[:,2]=np.arctan2(np.sin(difference[:,2]),np.cos(difference[:,2]))
            diffs.append(dict(iteration=i,max_translation_difference_m=float(np.abs(difference[:,:2]).max()),max_yaw_difference_rad=float(np.abs(difference[:,2]).max())))
        write_csv(out/'matlab_cpp_steps.csv',diffs)
        (out/'matlab_cpp_steps.json').write_text(json.dumps(diffs,indent=2))
        text+='<h2>原实现与错误定位</h2><p>参考源码在 Downloads/2D_Occupancy_SLAM_MATLAB。当前 Main_OGSLAM.m 含活动语句 Pose=Pose_GT，loadParam.m 为单层 0.1 m、DOWN_TIME=0、最多 20 步；这份保存状态不能直接充当噪声初值实验。本轮保持原文件只读，用独立入口传入真实噪声里程计或提供的 Cartographer 初值，GT 只做评价。MATLAB 参考副本仅去除图形/无用 Info 输出，并将 parfor 串行执行；调用原数值函数。</p>'
        text+='<p>旧测试另外使用 0.2 m 单层、10 步、CG 200/0.05，并将原始自由采样按网格重建；与这份 MATLAB 活动路径不同。本轮保留全部原始 Scan.xy/Odd，采用相同固定网格、初始 round/迭代 N floor、完整网格 N 平滑权重 10、活动地图权重 1e-7 和直接求解。原脚本 DOWN_TIME=0 使首步就使用 SecondSmoothWeigh 和 MinMeanDeltaSecond；没有把未使用的 FirstSmoothWeigh 当作实际权重。</p>'
        if diffs:
            text+=f'<p>{identity["scene"]}/{identity["case"]} 的 {len(diffs)} 个实际迭代状态逐步比较：最大平移差 {max(d["max_translation_difference_m"] for d in diffs):.4g} m，最大角度差 {max(d["max_yaw_difference_rad"] for d in diffs):.4g} rad。<a href="matlab_cpp_steps.csv">完整逐步差值</a>。这证明该输入和路径上的数值一致性，不外推所有模式或未提供的论文参数。</p>'
        text+='<p>前一轮独立 MATLAB 已直接读取原始 MAT 真值，核验旧 41 组 initial/joint 共 82 份评价，最大差 2.22e-15。那证明欧氏 RMSE 算式正确，不意味着旧优化配置等价。正式 C++ 入口另已恢复 ModeMulti 配置传递、选区阶段、N 从零重算和求解参数；该两阶段能力通过独立集成检查，本轮主表使用这里列出的单层 MATLAB 配置。</p>'
    text+='<h2>输入、参数与评价口径</h2><p>每场景 264／268／364／372 帧，每帧 1081 射线。每场景五份 noisy odometry 和五份 Cartographer 初值；Simu2680 的额外 Noise_24 单列，共 41 组。缺首帧 Pose 的输入显式跳过首帧扫描和对应评价行，不以 GT 补位姿。四个场景上的初值重复不是 41 个独立场景。</p>'
    keys=['ModeMulti','Scale','DownRate','DownTime','MaxIter','MinMeanDeltaFirst','MinMeanDeltaPoseFirst','MapSmoothingWeightSecond','WeightSmoothNSecond','ModeSparseSolver','SolverFirstMaxIter','SolverFirstTolerance','WeightO','WeightSmoothN','MapSmoothingWeightFirst','MinDelta','MinDeltaPose','FixedGrid','InitialMapRound','NormalizationRound']
    text+=table(['参数','实际值'],[[k,params[k]] for k in keys if k in params])
    text+='<p>观测来源：'+html.escape(protocol.get('samples','endpoint resampling'))+'。原始数据和旧实验结果保留。共同配置源和实际 argv 存于各运行目录。</p>'
    text+='<p>主位置指标为 sqrt(mean(dx²+dy²))，单位 m；航向误差回绕后计算 RMSE，单位 °。固定首帧不计入绝对误差。所有结果共用由 initial 首帧确定的 SE(2)，不分别拟合最终轨迹，不拟合尺度。原 MATLAB FuncEval 的位置指标为 (RMSE_x+RMSE_y)/2，另存 matlab_axis_mean_rmse_m；sqrt(mean([dx²,dy²])) 也独立保存，三者不可混称。joint/reintegrated 使用完全相同的最终位姿。</p>'
    pose_rows=[]
    for row in grouped:
        pose_rows.append([row['scene'],'Odometry' if row['kind']=='noisy_odometry' else 'Cartographer',*[f"{row[m+'_'+k+'_mean']:.5f}" for k in ('translation_rmse_m','heading_rmse_deg') for m in ('initial','joint')]])
    text+='<h2>全部轨迹结果</h2>'+table(['场景','初值','初始位置 m','最终位置 m','初始航向 °','最终航向 °'],pose_rows)
    text+='<p>主表为五次独立初值运行的 RMSE 均值。<a href="all_trajectory_metrics.csv">全部逐组 CSV</a> · <a href="trajectory_grouped.csv">均值与标准差</a> · <a href="extra_initialization.csv">额外 Noise_24</a> · <a href="trajectory_table.tex">LaTeX 表</a></p><img src="pose_errors_all.png" alt="全部初值的前后误差">'
    previous=sources.get('previous_results')
    if previous:
        prior={(r['scene'],r['case']):r for r in json.loads((Path(previous)/'metrics.json').read_text())};paired=[]
        for row in runs:
            old=prior[row['scene'],row['case']];paired.append(dict(scene=row['scene'],case=row['case'],previous_position_rmse_m=old['joint_translation_rmse_m'],aligned_position_rmse_m=row['joint_translation_rmse_m'],previous_heading_rmse_deg=old['joint_heading_rmse_deg'],aligned_heading_rmse_deg=row['joint_heading_rmse_deg']))
        write_csv(out/'previous_vs_aligned.csv',paired);text+='<p><a href="previous_vs_aligned.csv">与旧测试逐组配对</a>。这里比较的是观测、网格、求解和预算一起恢复后的流程；不将变化归因于单一参数。</p>'
    regressions=[[r['scene']+'/'+r['case'],f"{r['initial_translation_rmse_m']:.5f} → {r['joint_translation_rmse_m']:.5f}",f"{r['initial_heading_rmse_deg']:.5f} → {r['joint_heading_rmse_deg']:.5f}"] for r in runs if not r['translation_improved'] or not r['heading_improved']]
    text+='<h2>保留的退化与限制</h2>'+table(['case','位置 RMSE m','航向 RMSE °'],regressions)
    text+='<p>迭代达到上限不等于收敛。这里未改变迭代预算来追逐较好的 GT 分数，也未删除退化初值。当前保存的 MATLAB 配置是代码一致性参考；尚不能据此宣称精确复现另一份未提供的论文实验配置。</p>'
    text+='<h2>三种原生地图</h2><p>initial 使用输入位姿；joint 是直接优化的 M/N；reintegrated 将全部原始观测按最终位姿从空场积分。几何真值来自原 MAT 的 Fourier 边界、矩形及外边界，按无噪声 Raw 验证解释。对 M−logit(p)N 的零集做 marching-squares 近似，预测按 0.01 m、真值按最多 0.005 m 采样。评价预测到全部真值、可见真值到预测的双向距离，各方向截断 1 m 后平均；空预测罚 1 m。可见支持由原始 GT 射线确定，不用优化结果选择支持，无地图 ICP。</p><p>同时报告 p=0.50 与 0.65；前者是 M 零面，后者对应输出占据阈值。网格为实际原生分辨率，独立高分辨率显示图不进入几何评分。</p>'
    text+=table(['场景','初值','p','initial','joint','reintegrated'],[[r['scene'],r['kind'],r['probability_threshold'],*[f"{r[m+'_symmetric_capped_mae_m_mean']:.5f}" for m in ('initial','joint','reintegrated')]] for r in grouped_maps])
    text+=table(['p','joint 优于 initial','reintegrated 优于 initial','joint 优于 reintegrated'],[[p,c['joint_better_than_initial_symmetric_capped_mae_m'],c['reintegrated_better_than_initial_symmetric_capped_mae_m'],c['joint_better_than_reintegrated_symmetric_capped_mae_m']] for p,c in summary['map_comparisons'].items()])
    text+='<p><a href="all_map_metrics.csv">全部地图 CSV</a> · <a href="map_grouped.csv">地图分组指标</a>；包含未截断误差、P95、precision/recall/F1。</p><img src="map_errors_grouped.png" alt="三类地图两种阈值的定量误差">'
    text+=f'<h2>成本与可视化</h2><p>C++ 进程耗时范围 {summary["per_run_wall_seconds_range"][0]:.1f}–{summary["per_run_wall_seconds_range"][1]:.1f} s，包括输入和输出；各次运行实际并行，时间合计不是整批经过时间。VmHWM 采样峰值最大 {summary["peak_rss_gib_max_sampled"]:.3f} GiB，VmSwap 采样最大 {summary["peak_swap_kib_max_sampled"]} KiB，和各次耗时配对保存。</p>'
    text+=f'<p>两档阈值共 {6*len(runs)} 份地图评价，其中 {len(summary["empty_contours"])} 份没有对应概率等值线：未截断距离记为缺失，capped 指标按 1 m 计、F1 为 0。分组表保留 defined_count；不删除这些记录或将缺失距离计为 0。</p>'
    text+='<p>每场景固定展示 carto_1，未按效果挑选。RViz 标准 OccupancyGrid 的 Topic 可切换 /occupancy_slam/{initial,optimized,reintegrated}/map_display；初始/重建显示分辨率可选 0.05 或 0.1 m，joint 保留变量网格。</p><img src="trajectories_fixed_carto1.png" alt="固定初值的轨迹对照"><img src="native_maps_fixed_carto1.png" alt="原生场与独立几何"><p><a href="sources.json">运行索引</a> · <a href="summary.json">完整汇总</a>。可复现命令和实现检查见工程 docs/VALIDATION.md。</p></html>'
    (out/'REPORT.html').write_text(text)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('results', 'maps', 'converted'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--wait', action='store_true')
    p.add_argument('--matlab-reference',type=Path)
    p.add_argument('--previous-results',type=Path)
    a = p.parse_args()
    out = Path(os.environ['SDF_BA_RUN_ROOT'])
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
        matlab_reference=str(a.matlab_reference) if a.matlab_reference else None,previous_results=str(a.previous_results) if a.previous_results else None,representative_case='carto_1 in every scene, fixed index; not best result'), indent=2)+'\n')
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
    report(out,grouped,grouped_maps,summary,runs)
    print(json.dumps(summary,indent=2),flush=True)


if __name__ == '__main__':
    main()
