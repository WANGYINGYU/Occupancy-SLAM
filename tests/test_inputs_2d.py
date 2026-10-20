#!/usr/bin/env python3
"""Real TXT backend runs and renamed-topic ROS1 bags through actual slam_toolbox."""
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
from make_fixtures import make_bags
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from pipeline_config import parse_config
from rosbag_pipeline import parser as bag_parser
from rviz_viewer import parser as viewer_parser, load_scene

run=Path(os.environ['SDF_BA_RUN_ROOT'])
results={}


def invoke(name,command,expected=True,contains=None):
    with (run/(name+'.log')).open('w') as log:
        child=subprocess.run([str(x) for x in command],stdout=log,stderr=subprocess.STDOUT,cwd=run)
    text=(run/(name+'.log')).read_text()
    assert (child.returncode==0)==expected,(name,child.returncode,text[-4000:])
    if contains: assert contains in text,(name,text)
    print(name,'PASS',flush=True)


def check_txt(name,dataset,profile,pose_source):
    out=run/name
    extra=[]
    if pose_source=='odom':
        empty=run/'no_pose_file';empty.mkdir()
        extra=['--dataset',empty,'--ranges',ROOT/'Data'/dataset/'Range.txt',
               '--odom',ROOT/'Data'/dataset/'Odom.txt','--pose-source','auto']
    invoke(name,['bash',ROOT/'tools/run_2d.sh','txt','--dataset',ROOT/'Data'/dataset,
                 '--backend-config',ROOT/'config/backend'/profile,'--pose-source',pose_source,
                 '--output',out,'--start',2,'--scan-stride',3,'--max-scans',8,'--iterations',2,'--resolution',.5]+extra)
    summary=json.loads((out/'summary.json').read_text())
    assert summary['accepted_scans']==8
    indices=np.loadtxt(out/'selected_indices.txt',dtype=int)
    np.testing.assert_array_equal(indices,np.arange(2,26,3))
    np.testing.assert_array_equal(np.loadtxt(out/'Range.txt'),np.loadtxt(ROOT/'Data'/dataset/'Range.txt')[indices])
    source_poses=np.loadtxt(ROOT/'Data'/dataset/'Pose.txt')
    if pose_source=='file':
        np.testing.assert_array_equal(np.loadtxt(out/'Pose.txt'),source_poses[indices])
    if (ROOT/'Data'/dataset/'Odom.txt').exists():
        original=np.loadtxt(ROOT/'Data'/dataset/'Odom.txt')
        # Independently compose the selected odometry using scalar SE2 formulas.
        absolute=[original[0].copy()]
        for dx,dy,theta in original[1:]:
            x,y,a=absolute[-1]
            absolute.append(np.array([x+np.cos(a)*dx-np.sin(a)*dy,y+np.sin(a)*dx+np.cos(a)*dy,a+theta]))
        absolute=np.array(absolute)
        if pose_source=='odom':
            expected=absolute[indices].copy()
            expected[:,2]=np.arctan2(np.sin(expected[:,2]),np.cos(expected[:,2]))
            np.testing.assert_allclose(np.loadtxt(out/'Pose.txt'),expected,atol=1e-9)
        reduced=np.loadtxt(out/'Odom.txt')
        for i in range(1,len(indices)):
            a,b=absolute[indices[i-1]],absolute[indices[i]]
            d=b[:2]-a[:2]
            expected=[np.cos(a[2])*d[0]+np.sin(a[2])*d[1],-np.sin(a[2])*d[0]+np.cos(a[2])*d[1],
                      np.arctan2(np.sin(b[2]-a[2]),np.cos(b[2]-a[2]))]
            np.testing.assert_allclose(reduced[i],expected,atol=1e-9)
    backend=json.loads((out/'optimized/summary.json').read_text())
    assert backend['frames']==8 and backend['iterations']>0
    scene=load_scene(viewer_parser().parse_args(['2d','--result',str(out)]))
    assert len(scene['maps'])==3 and scene['maps']['initial']['display_grid']['resolution']==.05
    results[name]=dict(summary=summary,backend=backend,viewer_load=True)


# YAML defaults, CLI precedence, config-relative paths, typo/type failures.
profile=run/'input.yaml'
profile.write_text(yaml.safe_dump(dict(bag='input.bag',scan_topic='/custom/scan',iterations=3,slam_config='mapper.yaml')))
a=parse_config(bag_parser(),['--config',str(profile),'--iterations','2'],path_keys=('bag','backend','slam_config'))
assert a.iterations==2 and a.bag==run/'input.bag' and a.slam_config==run/'mapper.yaml'
for content in (dict(scan_topik='/typo'),dict(max_scans=2.5),dict(odom_source='typo')):
    profile.write_text(yaml.safe_dump(content))
    try: parse_config(bag_parser(),['--config',str(profile),'--bag','dummy'],path_keys=('bag','backend','slam_config'))
    except ValueError: pass
    else: raise AssertionError(content)
check_txt('txt_museum','Museum_b0','museum_b0.txt','file')
check_txt('txt_intel_pose_odom','Intel','intel.txt','file')
check_txt('txt_intel_odom_only','Intel','intel.txt','odom')

topics=dict(laser='/robot/front/ranges',cloud='/lidar/filtered_cloud',odom='/robot/wheel/odometry',
            tf='/robot/transforms',tf_static='/robot/static_transforms')
fixtures=run/'bags';fixtures.mkdir()
laser,cloud=make_bags(fixtures,topics,decoy=True)
for i,(name,bag,source,topic) in enumerate([
    ('renamed_laser_tf',laser,'tf',topics['laser']),
    ('renamed_laser_odom',laser,'odom',topics['laser']),
    ('renamed_cloud_tf',cloud,'tf',topics['cloud'])]):
    out=run/name
    settings=dict(bag=str(bag),scan_topic=topic,odom_source=source,odom_topic=topics['odom'],
                  tf_topic=topics['tf'],tf_static_topic=topics['tf_static'],odom_frame='odom',
                  tf_max_gap=1.1,iterations=1,resolution=.4,beam_stride=2,bins=360,max_range=15.,
                  scan_timeout=1.,domain_id=160+i,output=str(out),slam_config=str(ROOT/'config/slam_toolbox.yaml'))
    config=run/(name+'.yaml');config.write_text(yaml.safe_dump(settings))
    invoke(name,['bash',ROOT/'tools/run_2d.sh','bag','--config',config])
    summary=json.loads((out/'summary.json').read_text())
    assert summary['accepted_scans']>=20 and summary['input_topics']['scan']==topic
    np.testing.assert_allclose(np.loadtxt(out/'input_odometry.txt')[0],[.4,-.1,.3],atol=1e-9)
    assert (out/'slam_graph.posegraph').exists()
    effective_frontend=yaml.safe_load((out/'slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
    requested_frontend=yaml.safe_load((ROOT/'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert effective_frontend['resolution']==requested_frontend['resolution']
    assert effective_frontend['resolution']!=settings['resolution']
    assert json.loads((out/'run_status.json').read_text())['status']=='completed'
    assert json.loads((out/'optimized/summary.json').read_text())['iterations']>0
    results[name]=summary
np.testing.assert_allclose(np.loadtxt(run/'renamed_laser_tf/input_odometry.txt'),
                           np.loadtxt(run/'renamed_laser_odom/input_odometry.txt'),atol=1e-10)
np.testing.assert_allclose(np.loadtxt(run/'renamed_laser_tf/Range.txt'),
                           np.loadtxt(run/'renamed_cloud_tf/Range.txt'),atol=2e-6)
bad=run/'bad_topic.yaml'
settings.update(odom_source='odom',odom_topic='/does_not_exist',output=str(run/'bad_topic'))
bad.write_text(yaml.safe_dump(settings))
invoke('reject_wrong_odom',['bash',ROOT/'tools/run_2d.sh','bag','--config',bad],False,'No Odometry')
settings.update(scan_topic='/does_not_exist',odom_topic=topics['odom'],output=str(run/'bad_scan'))
bad.write_text(yaml.safe_dump(settings))
invoke('reject_wrong_scan',['bash',ROOT/'tools/run_2d.sh','bag','--config',bad],False,'No scan topic')
(run/'verification.json').write_text(json.dumps(results,indent=2))
print('All TXT and renamed-topic pipeline checks passed',flush=True)
