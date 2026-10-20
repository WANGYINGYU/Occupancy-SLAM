#!/usr/bin/env python3
"""Controlled linear-solve diagnostic; observations, objective and N unchanged."""
import csv,json,os,shutil,subprocess,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from evaluate_2d import evaluate
from project_paths import ROOT


def one(name,changes,source,out,gt):
    run=out/name;run.mkdir()
    for f in ['Endpoints.txt','Endpoints.npz','Pose.txt','timestamps.txt','gt_indices.txt']:
        (run/f).symlink_to(source/f)
    params=dict(line.split() for line in (source/'backend_config.txt').read_text().splitlines())
    params.update(changes)
    (run/'backend_config.txt').write_text(''.join(f'{k} {v}\n' for k,v in params.items()))
    cmd=[str(ROOT/'build/occupancy_slam_2d'),'--endpoints',str(run/'Endpoints.txt'),'--poses',str(run/'Pose.txt'),'--timestamps',str(run/'timestamps.txt'),'--config',str(run/'backend_config.txt'),'--output',str(run/'optimized'),'--max-observations','20000000','--step-policy','legacy']
    (run/'command.json').write_text(json.dumps(cmd,indent=2))
    start=time.monotonic();rss=0
    with (run/'backend.log').open('w') as log:
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        while p.poll() is None:
            try:
                fields=dict(l.split(':',1) for l in Path(f'/proc/{p.pid}/status').read_text().splitlines() if ':' in l)
                rss=max(rss,int(fields.get('VmHWM','0 kB').split()[0]))
            except FileNotFoundError:pass
            time.sleep(.2)
    status=dict(returncode=p.returncode,wall_seconds=time.monotonic()-start,peak_rss_kib_sampled=rss)
    (run/'run_status.json').write_text(json.dumps(dict(status='completed' if p.returncode==0 else 'failed',**status),indent=2))
    if p.returncode:raise RuntimeError(f'{name} failed: '+(run/'backend.log').read_text()[-500:])
    m=evaluate(run,gt,run/'evaluation',run/'gt_indices.txt')
    result=dict(name=name,changes=changes,**status,methods=m['methods'],summary=json.loads((run/'optimized/summary.json').read_text()))
    (run/'audit.json').write_text(json.dumps(result,indent=2))
    print(name,m['change'],flush=True)
    return result


def main():
    base=Path('/media/yingyuwang/Data/experiments/sdf_ba/codex_runs')
    source=base/'occupancy_matlab_all_20261020_01/Simu2680/odom_1'
    gt=base/'occupancy_matlab_conversion_20261020_01/converted/Simu2680/Pose_GT.txt'
    out=Path(os.environ['SDF_BA_RUN_ROOT'])
    shutil.copy2(__file__,out/'audit_source.py')
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures=[pool.submit(one,n,c,source,out,gt) for n,c in [('direct',{'ModeSparseSolver':'false'}),('cg_tight',{'SolverFirstMaxIter':'2000','SolverFirstTolerance':'1e-8'})]]
        rows=[f.result() for f in futures]
    (out/'results.json').write_text(json.dumps(rows,indent=2))

if __name__=='__main__':main()
