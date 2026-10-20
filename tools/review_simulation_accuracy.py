#!/usr/bin/env python3
"""Create the final evidence index from complete, explicitly supplied simulation runs."""
import argparse,csv,html,json,os,shutil,time
from pathlib import Path
import numpy as np
from scipy.io import loadmat

def rows(path):
    return json.loads((path/'metrics.json').read_text())
def table(head, body):
    return '<table><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in head)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in body)+'</table>'
def link(path,label):
    return '<a href="'+html.escape(str(path),quote=True)+'">'+html.escape(label)+'</a>'
def csv_file(path,data):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in data for k in r)));w.writeheader();w.writerows(data)
def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['single','short','extended','single_report','short_report','extended_report','matlab_reference','rviz']:
        p.add_argument('--'+k.replace('_','-'),type=Path,required=True)
    p.add_argument('--wait',action='store_true');a=p.parse_args();out=Path(os.environ['SDF_BA_RUN_ROOT'])
    while True:
        ready=all((getattr(a,k)/'REPORT.html').exists() for k in ['single_report','short_report','extended_report'])
        if ready:break
        if not a.wait:raise ValueError('All three completed reports are required')
        time.sleep(5)
    data={k:rows(getattr(a,k)) for k in ['single','short','extended']}
    identities=None
    for name,rs in data.items():
        assert len(rs)==41 and all(r['status']=='completed' for r in rs),name
        ids={(r['scene'],r['case']) for r in rs}
        if identities is None:identities=ids
        assert identities==ids
    lookup={k:{(r['scene'],r['case']):r for r in v} for k,v in data.items()}
    combined=[]
    for scene,case in sorted(identities):
        row=dict(scene=scene,case=case,initial_position_rmse_m=lookup['extended'][scene,case]['initial_translation_rmse_m'])
        for name in data:
            r=lookup[name][scene,case]
            row.update({name+'_position_rmse_m':r['joint_translation_rmse_m'],name+'_heading_rmse_deg':r['joint_heading_rmse_deg'],name+'_iterations':r['iterations'],name+'_stop':r['stop_reason']})
        combined.append(row)
    csv_file(out/'paired_profiles.csv',combined)
    reference=json.loads((a.matlab_reference/'completion.json').read_text());case=a.single/reference['scene']/reference['case'];diffs=[]
    for f in sorted(a.matlab_reference.glob('Pose_*.mat'),key=lambda f:int(f.stem.split('_')[1])):
        i=int(f.stem.split('_')[1]);d=np.loadtxt(case/'optimized/steps/fine'/str(i)/'poses.txt')-loadmat(f)['Pose'];d[:,2]=np.arctan2(np.sin(d[:,2]),np.cos(d[:,2]))
        diffs.append(dict(iteration=i,max_coordinate_difference_m=float(abs(d[:,:2]).max()),max_translation_vector_difference_m=float(np.linalg.norm(d[:,:2],axis=1).max()),max_heading_difference_rad=float(abs(d[:,2]).max())))
    assert len(diffs)==20
    csv_file(out/'matlab_cpp_20_steps.csv',diffs)
    summaries={k:json.loads((getattr(a,k+'_report')/'summary.json').read_text()) for k in data}
    ext=data['extended'];position=[r['joint_translation_rmse_m'] for r in ext];heading=[r['joint_heading_rmse_deg'] for r in ext]
    result=dict(cases=41,translation_improved=sum(r['translation_improved'] for r in ext),heading_improved=sum(r['heading_improved'] for r in ext),final_position_rmse_range_m=[min(position),max(position)],final_heading_rmse_range_deg=[min(heading),max(heading)],matlab_reference_steps=20,max_matlab_cpp_coordinate_difference_m=max(r['max_coordinate_difference_m'] for r in diffs),max_matlab_cpp_translation_vector_difference_m=max(r['max_translation_vector_difference_m'] for r in diffs),max_matlab_cpp_heading_difference_rad=max(r['max_heading_difference_rad'] for r in diffs),sources={k:str(getattr(a,k)) for k in vars(a) if isinstance(getattr(a,k),Path)},profile_summaries={k:{field:v[field] for field in ['cases','translation_improved','heading_improved','stops','map_comparisons','per_run_wall_seconds_range','peak_rss_gib_max_sampled','peak_swap_kib_max_sampled']} for k,v in summaries.items()})
    (out/'verification.json').write_text(json.dumps(result,indent=2)+'\n');shutil.copy2(__file__,out/'report_source.py')
    body='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Occupancy-SLAM simulation 精度复核最终报告</title><style>body{font:16px/1.7 system-ui;max-width:1200px;margin:32px auto;padding:0 24px;color:#243039}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:right}td:first-child,th:first-child{text-align:left}img{max-width:100%}.result{background:#eaf4ef;border-left:4px solid #328364;padding:18px}a{color:#12669b}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f5f6;padding:16px}</style><h1>二维 simulation：精度问题定位、修复与全量验证</h1>'
    body+=f'<p class="result">四个场景的全部 41 个初值均完成。统一较充足预算的粗到细流程：位置 RMSE 改善 {result["translation_improved"]}/41，航向 RMSE 改善 {result["heading_improved"]}/41；最终位置 RMSE 为 {min(position)*100:.3f}–{max(position)*100:.3f} cm，航向 RMSE 为 {min(heading):.4f}–{max(heading):.4f}°。所有数值取最终接受状态，GT 不参与优化或停止判断。</p>'
    body+='<h2>此前误差大的原因</h2><p>先前的 0.2 m 单层、10 次迭代、CG 200/0.05 测试没有忠实运行原完整流程：批量入口强制关闭多分辨率；MAT 数据转换按端点和网格间距重建自由空间样本，改变了原始观测；默认线性求解预算偏低。另外，MATLAB 当前保存配置的初始化分箱、N 平滑和活动地图正则与旧 C++ 测试设置不一致。旧误差算式经原 MATLAB 独立复核，82 份轨迹最大差 2.22e-15；问题主要在测试流程和实现配置，不应把旧结果解释为算法精度。</p>'
    body+='<p>已增加无损 Scan.xy/Odd 二进制入口、恢复粗到细及选区优化、落实两阶段求解参数、独立设置初始分箱和 N 平滑权重。新累计法方程与原 Jacobian 矩阵乘积核对。N 的 pose 导数仍省略，在每次接受迭代后从零累计并平滑，遵循论文近似。没有用真值替换噪声初值。</p>'
    body+='<h2>原 MATLAB 数值一致性</h2><p>Downloads 中 Main_OGSLAM.m 当前含 Pose=Pose_GT；未修改原文件，参考入口改为显式加载提供的噪声初值。loadParam.m 的活动路径是 0.1 m 单分辨率、20 步上限；DOWN_TIME=0 使首步即使用 SecondSmoothWeigh=1e-7，完整网格 N 平滑权重为 10。仅在参考副本移除图形与无用 Info 输出、将 parfor 串行化，数值函数保持原样。</p>'
    body+=f'<p>Simu2680/odom_1 完整 20 步对照：坐标分量最大差 {result["max_matlab_cpp_coordinate_difference_m"]:.4g} m，平移向量最大差 {result["max_matlab_cpp_translation_vector_difference_m"]:.4g} m，航向最大差 {result["max_matlab_cpp_heading_difference_rad"]:.4g} rad。'+link(out/'matlab_cpp_20_steps.csv','逐步差值 CSV')+'。这证明该完整单层路径的等价，不宣称不同配置的粗到细过程与 MATLAB 逐步相同。</p>'
    body+='<h2>统一配置下的最终精度</h2><p>同一份 simulation_multires_extended.txt 用于全部初值：精细网格 0.1 m，粗网格 0.5 m；最多 40 次粗阶段、10 次精细阶段，沿用既有步长阈值提前停止。原始观测和所有权重不变。主表为每种来源五份初值的 RMSE 均值，额外 Noise_24 单列；共四个场景，不将 41 份初值当成 41 个独立场景。</p>'
    grouped=[]
    for scene in sorted({r['scene'] for r in ext}):
        for kind in ['noisy_odometry','cartographer']:
            rs=[r for r in ext if r['scene']==scene and r['kind']==kind and r['case']!='odom_24']
            grouped.append([scene,'Odometry' if kind=='noisy_odometry' else 'Cartographer',f'{np.mean([r["initial_translation_rmse_m"] for r in rs]):.4f}',f'{100*np.mean([r["joint_translation_rmse_m"] for r in rs]):.3f}',f'{np.mean([r["initial_heading_rmse_deg"] for r in rs]):.4f}',f'{np.mean([r["joint_heading_rmse_deg"] for r in rs]):.4f}'])
    body+=table(['场景','初值','初始位置 m','最终位置 cm','初始航向 °','最终航向 °'],grouped)
    extra=next(r for r in ext if r['case']=='odom_24');body+=f'<p>额外 Simu2680/odom_24：{extra["initial_translation_rmse_m"]:.4f} m → {100*extra["joint_translation_rmse_m"]:.3f} cm；航向 {extra["joint_heading_rmse_deg"]:.4f}°。</p>'
    body+='<p>位置指标 sqrt(mean(dx²+dy²))；航向回绕后 RMSE，单位 °。初始首帧确定共同 SE(2)，所有最终状态共享该变换；不拟合最终轨迹、不拟合尺度，固定首帧不计入误差。原 MATLAB FuncEval 的 (RMSE_x+RMSE_y)/2 另列，避免把两种口径混用。</p>'
    body+='<h2>保留全部配置对照与负结果</h2>'
    comparisons=[]
    for name,label in [('single','MATLAB 活动单层 / 20 步'),('short','粗到细 / 8+3 上限'),('extended','粗到细 / 40+10 上限')]:
        rs=data[name];ss=summaries[name];comparisons.append([label,len(rs),f'{min(r["joint_translation_rmse_m"] for r in rs)*100:.3f}–{max(r["joint_translation_rmse_m"] for r in rs)*100:.3f}',str(ss['stops'])])
    body+=table(['配置','实际完成','最终位置 RMSE 范围 cm','停止原因'],comparisons)
    short=lookup['short']['Simu3640','odom_2'];extended=lookup['extended']['Simu3640','odom_2']
    body+=f'<p>Simu3640/odom_2 的短预算结果为 {short["joint_translation_rmse_m"]:.6f} m，粗细阶段均耗尽预算；仅提高预算的诊断降至厘米级后，对所有 41 组统一重跑，最终该例 {extended["joint_translation_rmse_m"]:.6f} m、{extended["iterations"]} 次更新。新预算是在观察到此问题后选定，属于本批次诊断修复验证，未经新场景独立验证。没有隐藏单层结果或挑选 GT 最优中间步。</p>'
    body+='<p>'+ ' · '.join(link(getattr(a,k+'_report')/'REPORT.html',label) for k,label in [('single','单层参考完整报告'),('short','短预算完整报告'),('extended','最终完整报告')])+' · '+link(out/'paired_profiles.csv','全部 41 组逐项配对')+'</p>'
    body+='<h2>联合变量地图与最终 pose 重建</h2><p>三类地图都独立保存：initial、joint、reintegrated。重建从空场使用全部原始样本和最终 pose，joint/reintegrated 的 pose 完全相同。地图与独立原始 simulation 几何比较，不以显示截图代替定量评价；原生网格等值线采用 p=0.50/0.65，报告双向距离、F1 和缺失情况。</p>'
    cmp=summaries['extended']['map_comparisons']['0.5']
    body+=f'<p>p=0.50 时，joint/reintegrated 的 capped 双向距离分别在 {cmp["joint_better_than_initial_symmetric_capped_mae_m"]}/41、{cmp["reintegrated_better_than_initial_symmetric_capped_mae_m"]}/41 组优于初始；joint 在 {cmp["joint_better_than_reintegrated_symmetric_capped_mae_m"]}/41 组优于同 pose 重建。因此轨迹高精度不等于联合变量地图在每个指标上都优于重建。p=0.65 下某些场没有等值线，保留缺失距离、1 m capped 惩罚和 F1=0；该阈值也保留在完整报告。</p>'
    body+='<h2>真实 RViz 可视化</h2><p>标准 nav_msgs/msg/OccupancyGrid，通过 Map → Topic 切换 /occupancy_slam/{initial,optimized,reintegrated}/map_display。初始与重建显示 0.05 m，优化变量图 0.1 m；黑灰白配色，同一相机与 map 坐标系。下面是实际 RViz 窗口截图，非渲染示意图。</p>'
    for kind,label in [('initial','优化前'),('reintegrated','最终 pose 重建'),('optimized','联合优化变量')]:
        body+='<p>'+link(a.rviz/(kind+'.png'),label)+'</p><img src="'+str(a.rviz/(kind+'.png'))+'" alt="'+label+'">'
    body+='<pre>'+html.escape('cd /home/yingyuwang/Documents/Occupancy-SLAM\n./tools/view_rviz.sh 2d --result '+str(a.extended/'Simu2680/odom_1')+' --display-resolution 0.05')+'</pre>'
    body+='<h2>检查与复现</h2><p>7/7 单元与集成检查、二维 TXT/三维 PCD 架构集成检查、额外带注释参数显示导出检查通过；真实 RViz Topic 切换经 DDS 订阅、固定相机、保存配置和截图检查。二维 RViz 保留已知启动 shader 诊断，未出现持续绘制错误或崩溃。所有运行位于数据盘，源码、参数、实际 argv、输出与日志分别留存。N 导数未增加；没有修改原 Downloads 数据和 MATLAB 文件。</p>'
    body+='<p>耗时是在同机并发条件下逐次记录，不用来声明单任务性能；各次峰值 RSS 与 VmSwap 采样和该次运行配对。'+link(out/'verification.json','总索引与数据来源')+' · '+link(Path('/home/yingyuwang/Documents/Occupancy-SLAM/docs/VALIDATION.md'),'复现与检查记录')+'</p></html>'
    (out/'REPORT.html').write_text(body)
    print(json.dumps({k:v for k,v in result.items() if k not in ('sources','profile_summaries')},indent=2))
if __name__=='__main__':main()
