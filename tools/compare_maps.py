#!/usr/bin/env python3
"""Render three matched map estimators with common views and explicit differences."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from result_io import read_pcd

LABELS=['Before: initial poses','After: joint map variables','After: optimized-pose reintegration']
COLORS=['#3288ce','#e77522','#8d55a8']
METHODS=['initial','joint','reintegrated']


def probability(folder):
    m=np.loadtxt(folder/'occupancy.txt');n=np.loadtxt(folder/'normalization.txt')
    return 1/(1+np.exp(-np.clip(m/np.maximum(n,1e-9),-30,30)))


def save(fig,root,name):
    fig.savefig(root/(name+'.png'),dpi=170,facecolor='white')
    fig.savefig(root/(name+'.pdf'),facecolor='white')
    plt.close(fig)


def two(result,out):
    fields=[probability(result/m) for m in METHODS]
    poses=[np.loadtxt(result/m/'poses.txt') for m in METHODS]
    meta=yaml.safe_load((result/'map.yaml').read_text());x,y,_=meta['origin'];h=meta['resolution']
    extent=[x,x+fields[0].shape[1]*h,y,y+fields[0].shape[0]*h]
    center=poses[0][len(poses[0])//2,:2];half=3.
    fig,axes=plt.subplots(2,3,figsize=(18,10),layout='constrained')
    for col,(field,pose,label,color) in enumerate(zip(fields,poses,LABELS,COLORS)):
        for row in range(2):
            ax=axes[row,col];im=ax.imshow(field,origin='lower',extent=extent,cmap='gray_r',vmin=0,vmax=1,interpolation='nearest')
            ax.plot(pose[:,0],pose[:,1],color=color,lw=1.5)
            ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title=label+(' | full map' if row==0 else ' | same 6 m crop'))
            if row:ax.set(xlim=(center[0]-half,center[0]+half),ylim=(center[1]-half,center[1]+half))
    fig.colorbar(im,ax=axes[0,:],shrink=.6,label='Occupancy probability (same 0–1 scale)')
    fig.suptitle(f'2D | {len(poses[0])} matched scans | same grid | last two columns use identical final poses',fontsize=16)
    save(fig,out,'2d_three_maps')
    fig,axes=plt.subplots(1,3,figsize=(17,4.8),layout='constrained')
    diffs=[fields[1]-fields[0],fields[2]-fields[0],fields[1]-fields[2]]
    for ax,d,label in zip(axes,diffs,['Joint − initial','Reintegrated − initial','Joint − reintegrated']):
        im=ax.imshow(d,origin='lower',extent=extent,cmap='coolwarm',vmin=-1,vmax=1,interpolation='nearest')
        ax.set(title=label,xlabel='x (m)',ylabel='y (m)',aspect='equal')
    fig.colorbar(im,ax=axes,shrink=.8,label='Probability difference (no normalization per panel)')
    save(fig,out,'2d_probability_changes')
    return dict(frames=len(poses[0]),max_position_change_m=float(np.linalg.norm(poses[1][:,:2]-poses[0][:,:2],axis=1).max()),joint_vs_initial_probability_mae=float(np.abs(diffs[0]).mean()),reintegrated_vs_initial_probability_mae=float(np.abs(diffs[1]).mean()),joint_vs_reintegrated_probability_mae=float(np.abs(diffs[2]).mean()),crop_center_m=center.tolist(),crop_width_m=6.)


def three(result,out,threshold):
    data=[read_pcd(result/m/'occupancy_vertices.pcd') for m in METHODS]
    poses=[np.loadtxt(result/m/'trajectory.tum') for m in METHODS]
    xyz=data[0][0]
    for points,_ in data:np.testing.assert_array_equal(points,xyz)
    # Every panel uses identical coordinates, probability threshold, color range and camera.
    union=np.logical_or.reduce([p>=threshold for _,p in data])
    lo,hi=xyz[union].min(axis=0),xyz[union].max(axis=0)
    center=poses[0][len(poses[0])//2,1:3];half=4.
    fig=plt.figure(figsize=(18,10),layout='constrained')
    for i,((points,p),pose,label,color) in enumerate(zip(data,poses,LABELS,COLORS)):
        mask=p>=threshold
        ax=fig.add_subplot(2,3,i+1,projection='3d')
        im=ax.scatter(*points[mask].T,c=p[mask],cmap='viridis',vmin=threshold,vmax=1,s=.7,linewidths=0,rasterized=True,depthshade=False)
        ax.plot(pose[:,1],pose[:,2],pose[:,3],color=color,lw=1.4)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=f'{label}\n{mask.sum():,} vertices, p ≥ {threshold:g}')
        ax.set_box_aspect(hi-lo);ax.view_init(elev=35,azim=-75)
        ax=fig.add_subplot(2,3,i+4)
        ax.scatter(points[mask,0],points[mask,1],c=p[mask],cmap='viridis',vmin=threshold,vmax=1,s=1.5,rasterized=True)
        ax.plot(pose[:,1],pose[:,2],color=color,lw=1.5)
        ax.set(xlim=(center[0]-half,center[0]+half),ylim=(center[1]-half,center[1]+half),xlabel='x (m)',ylabel='y (m)',aspect='equal',title='Same 8 m crop | top view')
    fig.colorbar(im,ax=fig.axes,location='bottom',fraction=.02,pad=.03,aspect=50,label='Occupancy probability (same threshold and colors)')
    voxel=json.loads((result/'grid.json').read_text())['voxel']
    fig.suptitle(f'3D | {len(poses[0])} frames | shared {voxel:g} m grid | occupancy vertices, not a surface mesh',fontsize=16)
    save(fig,out,'3d_three_maps')
    p0,p1,p2=[p for _,p in data]
    fig,axes=plt.subplots(1,3,figsize=(17,5.2),layout='constrained')
    for ax,delta,label in zip(axes,[p1-p0,p2-p0,p1-p2],['Joint − initial','Reintegrated − initial','Joint − reintegrated']):
        im=ax.scatter(xyz[union,0],xyz[union,1],c=delta[union],cmap='coolwarm',vmin=-1,vmax=1,s=.7,rasterized=True)
        ax.set(title=label,xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)',aspect='equal')
    fig.colorbar(im,ax=axes,shrink=.8,label='Probability difference on the same vertices')
    save(fig,out,'3d_probability_changes')
    return dict(frames=len(poses[0]),max_position_change_m=float(np.linalg.norm(poses[1][:,1:4]-poses[0][:,1:4],axis=1).max()),source_vertices=len(xyz),threshold=threshold,displayed_vertices={m:int((p>=threshold).sum()) for m,(_,p) in zip(METHODS,data)},joint_vs_initial_probability_mae=float(np.abs(p1-p0).mean()),reintegrated_vs_initial_probability_mae=float(np.abs(p2-p0).mean()),joint_vs_reintegrated_probability_mae=float(np.abs(p1-p2).mean()),crop_center_m=center.tolist(),crop_width_m=8.)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('result2d','result3d','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--threshold',type=float,default=.6)
    a=p.parse_args()
    if not 0 <= a.threshold < 1: p.error('--threshold must be in [0, 1)')
    a.output.mkdir(exist_ok=False)
    metrics={'2d':two(a.result2d,a.output),'3d':three(a.result3d,a.output,a.threshold),'inputs':{'2d':str(a.result2d),'3d':str(a.result3d)},'ground_truth_evaluation':False}
    (a.output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(json.dumps(metrics,indent=2),flush=True)

if __name__=='__main__':main()
