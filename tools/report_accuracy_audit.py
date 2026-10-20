#!/usr/bin/env python3
"""Summarize controlled diagnostics without replacing the historical measurements."""
import csv,html,json,os,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_2d import pose_metrics,apply_alignment


def main():
    base=Path('/media/yingyuwang/Data/experiments/sdf_ba/codex_runs');out=Path(os.environ['SDF_BA_RUN_ROOT']);rows=[]
    for scene,case in [('Simu2680','odom_1'),('Simu2640','carto_1')]:
        original=base/'occupancy_matlab_all_20261020_01'/scene/case
        old=json.loads((original/'evaluation/metrics.json').read_text())
        gt=np.loadtxt(base/'occupancy_matlab_conversion_20261020_01/converted'/scene/'Pose_GT.txt')[old['matched_gt_rows']]
        align=old['shared_alignment'];key=scene+'_'+case
        paths=[('initial',original/'optimized/initial/poses.txt'),('previous_single_02_loose',original/'optimized/joint/poses.txt'),
               ('single_01_tight',base/'occupancy_accuracy_resolution_20261020_01'/key/'optimized/joint/poses.txt'),
               ('coarse_05_tight',base/'occupancy_accuracy_coarse_20261020_01'/key/'optimized/joint/poses.txt')]
        if scene=='Simu2680':
            paths += [(name,base/'occupancy_accuracy_solver_20261020_01'/folder/'optimized/joint/poses.txt') for name,folder in [('single_02_tight','cg_tight'),('single_02_direct','direct')]]
        paths += [(f'refine_01_step_{i}',base/'occupancy_accuracy_refinement_20261020_02'/key/'refined'/f'step_{i}/poses.txt') for i in range(1,4)]
        for label,path in paths:
            pose=np.loadtxt(path);pose=apply_alignment(pose,align['yaw_rad'],np.asarray(align['translation_m']))
            m=pose_metrics(pose,gt)[0]
            rows.append(dict(scene=scene,case=case,stage=label,translation_rmse_m=m['translation_m']['rmse'],heading_rmse_deg=m['heading_deg']['rmse'],coordinate_rmse_m=m['coordinate_rmse_m'],rpe1_translation_rmse_m=m['rpe_1_frame']['translation_m']['rmse'],poses=str(path)))
    with (out/'diagnostic_metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (out/'diagnostic_metrics.json').write_text(json.dumps(rows,indent=2))
    shutil.copy2(__file__,out/'report_source.py')
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),constrained_layout=True)
    stages=['initial','previous_single_02_loose','single_01_tight','coarse_05_tight','refine_01_step_3']
    labels=['Input','Previous\n0.2 m / loose','Single\n0.1 m / tight','Coarse\n0.5 m / tight','Coarse + fine\n0.5 to 0.1 m']
    for ax,scene in zip(axes,['Simu2680','Simu2640']):
        values=[next(r['translation_rmse_m'] for r in rows if r['scene']==scene and r['stage']==s) for s in stages]
        bars=ax.bar(np.arange(len(values)),values,color=['#999999','#c06c42','#5b97bd','#53758c','#237f61'])
        for bar,v in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,v+.015,f'{v:.4f}',ha='center',fontsize=9)
        ax.set_xticks(np.arange(5),labels,fontsize=8);ax.set_ylim(0,max(values)*1.15);ax.set_ylabel('Position RMSE (m)');ax.set_title(scene+(' / odom_1' if scene=='Simu2680' else ' / carto_1'));ax.grid(axis='y',alpha=.2)
    fig.suptitle('Controlled diagnosis: identical GT rows and shared first-pose frame; no N derivative')
    fig.savefig(out/'accuracy_diagnosis.png',dpi=180);fig.savefig(out/'accuracy_diagnosis.pdf');plt.close(fig)
    matlab=json.loads((base/'occupancy_accuracy_matlab_20261020_03/matlab_evaluation.json').read_text())
    table='<table><tr><th>场景 / 初值</th><th>阶段</th><th>位置 RMSE (m)</th><th>航向 RMSE (°)</th></tr>'
    for r in rows:
        table+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in [r['scene']+'/'+r['case'],r['stage'],f"{r['translation_rmse_m']:.6f}",f"{r['heading_rmse_deg']:.6f}"])+'</tr>'
    table+='</table>'
    report='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>二维精度复核：测试流程与求解精度</title><style>body{max-width:1100px;margin:36px auto;padding:0 20px;font:16px/1.75 system-ui;color:#243039}h1{font-size:28px}h2{font-size:22px;margin-top:32px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border-bottom:1px solid #ddd;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}img{max-width:100%}.note{background:#fff2dc;border-left:4px solid #bd7723;padding:15px}a{color:#135d94}</style>
<h1>二维 simulation 精度复核</h1><p>2026-10-20，针对上一轮大误差的独立诊断。</p><p class="note">已确认上一轮测试流程不足以作为完整算法的精度验证：批处理强制单分辨率，使用粗阶段宽松线性容差，且没有调用原选区细化阶段。旧 41 组数值是真实测量，误差公式已交叉核验；不能据此推断论文算法只有米级精度。本轮只完成下面两组受控诊断，尚未用原 MATLAB 流程重新完成 41 组。</p>
<h2>误差计算与数据对应</h2>'''
    report+=f'<p>实际运行 MATLAB R2026a，直接读取原始 Pose_GT MAT，对 41 组的 initial/joint 共 {len(matlab["rows"])} 份结果重算。位置 RMSE、按坐标平均 RMSE、航向 RMSE以及独立闭式 SE(2) 对齐结果，与 Python 最大差为 {matlab["max_difference"]:.3g}。不是把 Python 转换的 GT 再读一遍。</p>'
    report+='''<p>扫描核对覆盖四场景全部帧：前两场景转换端点与 Raw 对应帧逐坐标完全一致；后两场景与无噪声 Raw 的差为提供的约 0.02 m 每坐标噪声，未发现系统性平移。原工程 FuncEval 的平移 RMSE 按坐标平均，比本报告二维欧氏 RMSE 小 √2；此口径差异已明确列出，不能解释米级与厘米级的差距。</p>
<h2>定位到的流程差异</h2><p>src/apps/run2d.cpp 无条件设置 ModeMulti=false，tools/txt_pipeline.py 也强制关闭；原 Main_Occupancy_SLAM.cpp/FuncLeastSquares.cpp 的低分辨率阶段、选区高分辨率阶段与阶段停止条件没有进入该批处理路径。上一轮转换脚本显式用了 0.2 m 单层与 10 步；这不是原多分辨率流程。</p><p>线性残差满足 0.05 只意味着达到这个宽松阈值，不证明高精度位姿更新已充分求出。Simu2680/odom_1 固定相同输入、0.2 m 网格、目标、N 与十次迭代：旧解位置 RMSE 1.041743 m；CG 2000/1e-8 得到 0.077627 m，直接 LDLT 得到 0.077627 m。它们没有靠改变评价坐标系取得改善。原始单层结果经过额外独立刚性拟合可到 0.024974 m，这仅揭示整体偏差，未用于替换主指标。</p>
<h2>粗到细的受控诊断</h2><p>下表全部使用原先的 GT 行和 initial 首帧坐标变换，不另行对齐最终轨迹。0.5 m 粗阶段使用十次更新和 CG 2000/1e-8；0.1 m 精细阶段调用原选区残差/Jacobian，使用相同法方程充分求解并记录真实线性残差。精细阶段的三个提交状态全部保留；结论取第三步，不挑 GT 误差最小的中间状态。</p><p>N 的位姿导数始终不加入。原 FuncUpdateSelectN 是累加函数，因此诊断入口在每次调用前清零，然后按原函数重新计算和平滑；这符合“每次迭代后重算”的约定，未改变 N 导数近似。原 FuncSelectMapDelta 隐式将 CG 上限截为 floor(sqrt(n)) 且不报告残差，诊断入口使用相同矩阵并尊重配置上限。该候选用于定位缺失阶段的影响，不能在没有原二维 MATLAB 代码/参数逐步对照时声称是论文结果的精确复现。</p>'''
    report+=table+'<p><a href="diagnostic_metrics.csv">完整诊断 CSV</a></p><img src="accuracy_diagnosis.png" alt="固定评价方法下的精度对照">'
    report+='''<h2>正确性检查和边界</h2><p>新增已知双线性场、常数 N 的 Jacobian 检查，逐项核对位姿坐标／旋转和地图变量索引，通过 1e-8 容差。原 N 近似测试保留通过。这不是要求非恒定 N 情况下补 N 导数，也不是完整非线性精度证明。</p><p>本轮不修改生产优化器的导数，不覆盖原数据及旧 41 组结果，不根据 GT 调参或选择最优中间步。0.1 m 单层对 Simu2680/odom_1 仍为 0.372362 m，说明仅把网格变细不是完整修正。两组粗到细最终误差约 3.3–3.5 cm，只能支持这两组诊断，尚不能外推全部初值。</p><p>尚缺原二维 MATLAB 优化入口及 simulation 参数。当前仓库有二维 C++ 和三维 MATLAB，Downloads 只有这些二维 MAT 数据。需要按原实现对齐阶段、采样、权重、求解精度和停止条件后，再执行完整 41 组验收；不会把这两个较好的诊断结果替代它。</p>
<h2>证据</h2><ul><li><a href="../occupancy_accuracy_matlab_20261020_03/matlab_evaluation.csv">独立 MATLAB 全量误差核验</a></li><li><a href="../occupancy_accuracy_correspondence_20261020_01/all_frame_scan_checks.json">全部帧的扫描核对</a></li><li><a href="../occupancy_accuracy_solver_20261020_01/results.json">直接法／高精度 CG 同输入对照</a></li><li><a href="../occupancy_accuracy_resolution_20261020_01/results.json">0.1 m 单层对照</a></li><li><a href="../occupancy_accuracy_coarse_20261020_01/results.json">0.5 m 粗阶段</a></li><li><a href="../occupancy_accuracy_refinement_20261020_02/">精细阶段全部状态和线性残差</a></li><li><a href="../occupancy_accuracy_jacobian_20261020_01/">Jacobian 核验日志</a></li></ul><p>开发失败留存：MATLAB 首次因临时目录路径过长启动失败，改用数据盘短目录、无 JVM 模式；第二次结果容器初始化失败，改为 cell 收集后第三次成功。相关日志均保留；自动生成到 home 的本次 MATLAB 崩溃日志已移回数据盘。没有修改公式来绕过断言。</p></html>'''
    (out/'REPORT.html').write_text(report)
    historical=base/'occupancy_matlab_summary_20261020_02/REPORT.html'
    shutil.copy2(historical,out/'previous_REPORT.html')
    notice=f'<div class="note"><strong>后续精度复核：</strong>此处 41 组是单分辨率、宽松求解配置的实测，不足以作为完整粗到细算法的精度结论。已发现遗漏精细阶段及求解精度不足的影响。请先阅读 <a href="../{out.name}/REPORT.html">独立核验与诊断报告</a>；原始数值仍保留。</div>'
    historical.write_text(historical.read_text().replace('<h1>',notice+'<h1>',1))
    print('Saved',out/'REPORT.html')

if __name__=='__main__':main()
