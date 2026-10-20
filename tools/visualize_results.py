#!/usr/bin/env python3
"""Render matched before/after 2D and 3D scientific figures and an offline viewer."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
import plotly.graph_objects as go
from plotly.subplots import make_subplots


from result_io import read_pcd


def save_figure(fig,root,name):
    fig.savefig(root/f'{name}.png',dpi=170,facecolor='white')
    fig.savefig(root/f'{name}.pdf',dpi=170,facecolor='white')
    plt.close(fig)


def endpoints(ranges,poses,config):
    angles=np.linspace(float(config['MinAngle']),float(config['MaxAngle']),ranges.shape[1])
    local=np.stack([ranges*np.cos(angles),ranges*np.sin(angles)],axis=-1)
    good=(ranges>float(config['MinRange']))&(ranges<float(config['MaxRange']))
    c,s=np.cos(poses[:,2]),np.sin(poses[:,2])
    x=local[:,:,0]*c[:,None]-local[:,:,1]*s[:,None]+poses[:,0,None]
    y=local[:,:,0]*s[:,None]+local[:,:,1]*c[:,None]+poses[:,1,None]
    return np.column_stack([x[good],y[good]])


def probability(folder):
    raw=np.loadtxt(folder/'occupancy.txt')
    counts=np.loadtxt(folder/'normalization.txt')
    return 1/(1+np.exp(-np.clip(raw/np.maximum(counts,1e-9),-30,30)))


def plot2d(args,out):
    pipeline=args.pipeline
    config=dict(line.split() for line in (pipeline/'backend_config.txt').read_text().splitlines() if line.strip())
    before=np.loadtxt(pipeline/'Pose.txt');after=np.loadtxt(pipeline/'optimized/poses.txt')
    ranges=np.loadtxt(pipeline/'Range.txt')
    times=np.loadtxt(pipeline/'timestamps.txt')
    assert before.shape==after.shape and len(times)==len(before)==len(ranges)
    np.testing.assert_allclose(before[0],after[0],atol=1e-12)
    np.testing.assert_allclose(before,np.loadtxt(args.initial2d/'poses.txt'),atol=1e-12)
    points0,points1=endpoints(ranges,before,config),endpoints(ranges,after,config)
    lo=np.minimum(points0.min(axis=0),points1.min(axis=0))-.6
    hi=np.maximum(points0.max(axis=0),points1.max(axis=0))+.6
    field0,field1=probability(args.initial2d),probability(pipeline/'optimized')
    meta=yaml.safe_load((pipeline/'optimized/map.yaml').read_text())
    x,y,_=meta['origin'];res=meta['resolution']
    extent=[x,x+field1.shape[1]*res,y,y+field1.shape[0]*res]
    fig,axes=plt.subplots(2,2,figsize=(13,11),layout='constrained')
    for col,(pts,pose,field,label,color) in enumerate(zip([points0,points1],[before,after],[field0,field1],['slam_toolbox initialization','C++ occupancy optimization'],['#2767a5','#c95818'])):
        axes[0,col].scatter(pts[:,0],pts[:,1],s=.6,c='#212e3b',alpha=.55,rasterized=True)
        axes[1,col].imshow(field,origin='lower',extent=extent,cmap='gray_r',vmin=0,vmax=1,interpolation='nearest')
        for row in range(2):
            ax=axes[row,col]
            ax.plot(pose[:,0],pose[:,1],lw=1.4,color=color,label='trajectory')
            ax.scatter(pose[0,0],pose[0,1],marker='o',s=30,c='#23a873',edgecolors='white',linewidths=.6,zorder=5)
            ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)',aspect='equal')
            ax.set_title(('Matched scan endpoints' if row==0 else 'Continuous occupancy probability')+'\n'+label)
    fig.suptitle(f'Cartographer Revo LDS | {len(before)} matched scans | before / after',fontsize=16)
    save_figure(fig,out,'2d_map_comparison')
    summary=json.loads((pipeline/'optimized/summary.json').read_text())
    trace=np.genfromtxt(pipeline/'optimized/iterations.csv',delimiter=',',names=True)
    series=np.r_[trace['scan_sse'],summary['final_scan_sse']]
    displacement=np.linalg.norm(after[:,:2]-before[:,:2],axis=1)
    yaw=np.abs(np.arctan2(np.sin(after[:,2]-before[:,2]),np.cos(after[:,2]-before[:,2])))*180/np.pi
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout='constrained')
    axes[0].plot(before[:,0],before[:,1],color='#2767a5',label='slam_toolbox')
    axes[0].plot(after[:,0],after[:,1],color='#c95818',label='C++ optimized');axes[0].axis('equal')
    axes[0].set(title='Trajectory (no GT)',xlabel='x (m)',ylabel='y (m)');axes[0].legend()
    axes[1].plot(times-times[0],displacement,color='#7b50a2');axes[1].set(title='Position change from initialization',xlabel='Elapsed time (s)',ylabel='Displacement (m)')
    axes[2].plot(np.arange(len(series)),series,'o-',color='#2767a5');axes[2].set(title='Observation SSE (non-monotonic)',xlabel='Update count',ylabel='Scan SSE')
    for ax in axes:ax.grid(alpha=.2)
    save_figure(fig,out,'2d_diagnostics')
    stats=dict(summary,position_change_rms_m=float(np.sqrt(np.mean(displacement**2))),position_change_max_m=float(displacement.max()),yaw_change_max_deg=float(yaw.max()),duration_s=float(times[-1]-times[0]),hit_points=len(points0))
    stats.update(json.loads((pipeline/'summary.json').read_text()))
    return stats


def plot3d(args,out):
    initial,final=args.initial3d,args.optimized3d
    before=np.loadtxt(initial/'trajectory.tum');after=np.loadtxt(final/'trajectory.tum')
    np.testing.assert_allclose(before[:,0],after[:,0],atol=1e-8)
    np.testing.assert_allclose(before[0],after[0],atol=1e-10)
    if (initial/'selected_scans.txt').read_text()!=(final/'selected_scans.txt').read_text():raise ValueError('Different before/after scans')
    points0,_=read_pcd(initial/'optimized_points.pcd');points1,_=read_pcd(final/'optimized_points.pcd')
    if points0.shape!=points1.shape:raise ValueError('Different point count')
    occ0,prob0=read_pcd(initial/'occupancy_vertices.pcd');occ1,prob1=read_pcd(final/'occupancy_vertices.pcd')
    np.testing.assert_allclose(occ0,occ1,atol=1e-5)
    ids=np.linspace(0,len(points0)-1,min(70000,len(points0)),dtype=int)
    p0,p1=points0[ids],points1[ids]
    lo=np.minimum(points0.min(axis=0),points1.min(axis=0))
    hi=np.maximum(points0.max(axis=0),points1.max(axis=0))
    margin=np.maximum((hi-lo)*.025,.3);lo-=margin;hi+=margin
    zlo,zhi=np.quantile(np.r_[points0[:,2],points1[:,2]],[.01,.99])
    # Same selected point indices, world axes, height normalization and cameras.
    fig=plt.figure(figsize=(14,11),layout='constrained')
    for col,(points,pose,label,color) in enumerate(zip([p0,p1],[before,after],['Input odometry','Joint SE(3) + occupancy'],['#2767a5','#c95818'])):
        ax=fig.add_subplot(2,2,col+1)
        artist=ax.scatter(points[:,0],points[:,1],c=points[:,2],cmap='viridis',vmin=zlo,vmax=zhi,s=.5,rasterized=True)
        ax.plot(pose[:,1],pose[:,2],color=color,lw=1.4)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)',aspect='equal',title=label+' | top view')
        ax=fig.add_subplot(2,2,col+3,projection='3d')
        ax.scatter(points[:,0],points[:,1],points[:,2],c=points[:,2],cmap='viridis',vmin=zlo,vmax=zhi,s=.5,linewidths=0,rasterized=True,depthshade=False)
        ax.plot(pose[:,1],pose[:,2],pose[:,3],color=color,lw=1.4)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=label+' | 3D view')
        ax.set_box_aspect(hi-lo);ax.view_init(elev=30,azim=-65)
    fig.colorbar(artist,ax=fig.axes,location='bottom',fraction=.02,pad=.035,aspect=45,label='Height z (m)',extend='both')
    fig.suptitle(f'NC20 Short_Exp | {len(before)} frames | identical point subset and views',fontsize=16)
    save_figure(fig,out,'3d_cloud_comparison')
    summary=json.loads((final/'summary.json').read_text())
    trace=np.genfromtxt(final/'iterations.csv',delimiter=',',names=True)
    cost=np.r_[summary['initial_cost'],trace['cost_after']]
    shift=np.linalg.norm(after[:,1:4]-before[:,1:4],axis=1)
    angles=(Rotation.from_quat(before[:,4:]).inv()*Rotation.from_quat(after[:,4:])).magnitude()*180/np.pi
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout='constrained')
    axes[0].plot(before[:,1],before[:,2],'.-',label='Input odometry',color='#2767a5');axes[0].plot(after[:,1],after[:,2],'.-',label='Optimized',color='#c95818');axes[0].axis('equal');axes[0].legend();axes[0].set(title='Trajectory (no GT)',xlabel='x (m)',ylabel='y (m)')
    axes[1].plot(before[:,0]-before[0,0],shift,color='#7b50a2');axes[1].set(title='Position change from input',xlabel='Elapsed time (s)',ylabel='Displacement (m)')
    axes[2].plot(np.arange(len(cost)),cost,'o-',color='#2767a5');axes[2].set(title='Full joint objective',xlabel='Accepted update count',ylabel='Weighted sum of squared residuals')
    for ax in axes:ax.grid(alpha=.2)
    save_figure(fig,out,'3d_diagnostics')
    viewer=make_subplots(rows=1,cols=2,specs=[[{'type':'scene'},{'type':'scene'}]],subplot_titles=['Input odometry / 初始化','Joint optimization / 联合优化'])
    for col,(pts,pose,label) in enumerate(zip([p0,p1],[before,after],['Initial points','Optimized points']),1):
        viewer.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='markers',name=label,marker=dict(size=1.5,color=pts[:,2],colorscale='Viridis',cmin=zlo,cmax=zhi,opacity=.85),hoverinfo='skip'),row=1,col=col)
    for col,(pose,label,color) in enumerate(zip([before,after],['Input trajectory','Optimized trajectory'],['#2767a5','#c95818']),1):
        viewer.add_trace(go.Scatter3d(x=pose[:,1],y=pose[:,2],z=pose[:,3],mode='lines+markers',name=label,line=dict(width=5,color=color),marker=dict(size=2,color=color)),row=1,col=col)
    for col,(occ,prob,label) in enumerate(zip([occ0,occ1],[prob0,prob1],['Initial occupancy','Optimized occupancy']),1):
        mask=prob>=.6
        viewer.add_trace(go.Scatter3d(x=occ[mask,0],y=occ[mask,1],z=occ[mask,2],mode='markers',visible=False,name=label,marker=dict(size=2.5,color=prob[mask],cmin=.6,cmax=1,colorscale='YlOrRd'),hovertemplate='p=%{marker.color:.3f}<extra></extra>'),row=1,col=col)
    scene=dict(xaxis=dict(title='x (m)',range=[lo[0],hi[0]]),yaxis=dict(title='y (m)',range=[lo[1],hi[1]]),zaxis=dict(title='z (m)',range=[lo[2],hi[2]]),aspectmode='data',camera=dict(eye=dict(x=1.8,y=-1.8,z=1.3)))
    viewer.update_layout(scene=scene,scene2=scene,height=880,title=dict(text=f'NC20 Short_Exp · {len(before)} frames · drag to rotate / 拖动旋转，滚轮缩放',x=.5,xanchor='center',y=.985,font=dict(size=18)),margin=dict(l=25,r=25,b=60,t=150),legend=dict(orientation='h',y=-.06,x=.5,xanchor='center'),
                         updatemenus=[dict(type='buttons',direction='right',x=0,y=1.12,buttons=[dict(label='Point clouds / 点云',method='update',args=[{'visible':[True,True,True,True,False,False]}]),dict(label='Occupancy p ≥ 0.6 / 占据顶点',method='update',args=[{'visible':[False,False,True,True,True,True]}])])])
    post="""const graph=document.getElementById('{plot_id}'); let syncing=false;
    graph.on('plotly_relayout', ev => { if(syncing) return;
      const source=ev['scene.camera']?'scene':ev['scene2.camera']?'scene2':null;
      if(!source) return; syncing=true; const target=source==='scene'?'scene2':'scene';
      Plotly.relayout(graph, {[target+'.camera']:ev[source+'.camera']}).finally(()=>syncing=false);
    });"""
    viewer.write_html(out/'3d_interactive.html',include_plotlyjs=True,full_html=True,post_script=post,config={'responsive':True,'displaylogo':False})
    # Save exact plotted samples for audits; exported PCDs remain the complete clouds.
    np.savez_compressed(out/'plotted_3d_points.npz',indices=ids,initial=p0,optimized=p1)
    return dict(summary,height_color_limits_m=[float(zlo),float(zhi)],point_count=len(points0),plotted_point_count=len(ids),position_change_rms_m=float(np.sqrt(np.mean(shift**2))),position_change_max_m=float(shift.max()),rotation_change_max_deg=float(angles.max()),duration_s=float(before[-1,0]-before[0,0]),path_length_m=float(np.linalg.norm(np.diff(before[:,1:4],axis=0),axis=1).sum()))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('pipeline','initial2d','initial3d','optimized3d','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.size':11,'axes.titlesize':12,'axes.spines.top':False,'axes.spines.right':False})
    two=plot2d(args,out);three=plot3d(args,out)
    stats={'2d':two,'3d':three,'inputs':{k:str(v.resolve()) for k,v in vars(args).items()},'ground_truth_evaluation':False}
    (out/'metrics.json').write_text(json.dumps(stats,indent=2))
    report=f'''# Occupancy-SLAM 可视化测试结果\n\n真实 Cartographer Revo LDS bag 与 NC20 Short_Exp。所有前后图使用相同扫描、显示点集、坐标尺度和颜色范围。\n\n| 模式 | 帧数 | 初始目标 | 最终目标 | 最大位置改变量 |\n|---|---:|---:|---:|---:|\n| 二维 | {two['frames']} / {two['input_scans']} | {two['initial_scan_sse']:.3f} | {two['final_scan_sse']:.3f} | {two['position_change_max_m']:.3f} m |\n| 三维 | {three['frames']} | {three['initial_cost']:.3f} | {three['final_cost']:.3f} | {three['position_change_max_m']:.3f} m |\n\n二维列为观测 SSE，三维列为完整加权联合目标，不能互相比数值大小。二维迭代存在振荡，见诊断图。实际占据图仍有棋盘纹和边界晕影，不能视为地图质量已经达标；三维前后点云在整体尺度上的差别较小。位置改变量是相对于初始化的变化量，不是 GT 误差。没有独立 GT，本次不能证明地图／轨迹精度改善。\n\n- [二维地图前后对比](2d_map_comparison.png)\n- [二维轨迹与收敛](2d_diagnostics.png)\n- [三维点云前后对比](3d_cloud_comparison.png)\n- [三维轨迹与收敛](3d_diagnostics.png)\n- [交互式三维对比](3d_interactive.html)：离线打开，拖动旋转、滚轮缩放；两个视角同步；可切换点云与 p≥0.6 的占据网格顶点。\n\n每张静态图同时提供 PDF。交互点云显示 {three['plotted_point_count']} / {three['point_count']} 个确定性抽样点，完整点云见原始输出 PCD。占据网格顶点不是 mesh。\n\n实际输入路径与统计见 metrics.json；实际命令见各运行目录 launcher.json。\n'''
    (out/'REPORT.md').write_text(report)
    print(json.dumps(stats,indent=2))


if __name__=='__main__':main()
