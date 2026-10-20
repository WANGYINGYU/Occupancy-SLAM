#!/usr/bin/env python3
"""Run every converted MATLAB initialization using one common backend profile."""
import argparse,csv,json,os,shutil,subprocess,sys,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import numpy as np
from evaluate_2d import evaluate
from project_paths import ROOT


def run_case(source,case,run,options):
    scene=source.name; name=case['name']; out=run/scene/name;out.mkdir(parents=True)
    start=case['scan_start'];
    with np.load(source/'Endpoints.npz') as data: points=data['endpoints'][start:]
    rows=np.column_stack([np.repeat(np.arange(len(points)),points.shape[1]),points.reshape(-1,2)])
    np.savetxt(out/'Endpoints.txt',rows,fmt=['%d','%.17g','%.17g'])
    np.savez_compressed(out/'Endpoints.npz',endpoints=points)
    for f in ['Pose.txt','timestamps.txt','input_manifest.json']:shutil.copy2(source/name/f,out/f)
    shutil.copy2(options.backend_config or source/'backend_config.txt',out/'backend_config.txt')
    np.savetxt(out/'gt_indices.txt',np.arange(start,start+len(points)),fmt='%d')
    command=[str(ROOT/'build/occupancy_slam_2d'),'--endpoints',str(out/'Endpoints.txt'),
        '--poses',str(out/'Pose.txt'),'--timestamps',str(out/'timestamps.txt'),
        '--config',str(out/'backend_config.txt'),'--output',str(out/'optimized'),
        '--step-policy','legacy','--max-observations',str(options.max_observations),'--save-iterations','poses']
    if options.samples:
        command[1:3]=['--samples',str(options.samples/scene/'samples.bin'),'--sample-start',str(start)]
    (out/'backend_command.json').write_text(json.dumps(command,indent=2))
    (out/'run_status.json').write_text(json.dumps(dict(status='running')))
    before=time.monotonic();peak=0;swap=0
    with (out/'backend.log').open('w') as log:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        while proc.poll() is None:
            try:
                fields=dict((line.split(':',1)[0],line.split(':',1)[1].strip()) for line in Path(f'/proc/{proc.pid}/status').read_text().splitlines() if ':' in line)
                peak=max(peak,int(fields.get('VmHWM','0 kB').split()[0]));swap=max(swap,int(fields.get('VmSwap','0 kB').split()[0]))
            except FileNotFoundError:pass
            time.sleep(.2)
        elapsed=time.monotonic()-before
    status='completed' if proc.returncode==0 else 'failed'
    (out/'run_status.json').write_text(json.dumps(dict(status=status,returncode=proc.returncode,wall_seconds=elapsed,peak_rss_kib_sampled=peak,peak_swap_kib_sampled=swap),indent=2))
    row=dict(scene=scene,case=name,kind=case['kind'],frames=len(points),status=status,wall_seconds=elapsed,peak_rss_kib_sampled=peak,peak_swap_kib_sampled=swap)
    if proc.returncode:
        row['error']=(out/'backend.log').read_text()[-1000:];return row
    metrics=evaluate(out,source/'Pose_GT.txt',out/'evaluation',out/'gt_indices.txt')
    summary=json.loads((out/'optimized/summary.json').read_text())
    row.update(iterations=summary['iterations'],stop_reason=summary['stop_reason'],initial_cost=summary['initial_total_cost'],final_cost=summary['final_total_cost'])
    for method in ['initial','joint']:
        m=metrics['methods'][method]
        row[method+'_translation_rmse_m']=m['translation_m']['rmse'];row[method+'_heading_rmse_deg']=m['heading_deg']['rmse'];row[method+'_matlab_axis_mean_rmse_m']=m['matlab_axis_mean_rmse_m']
        row[method+'_rpe1_translation_rmse_m']=m['rpe_1_frame']['translation_m']['rmse'];row[method+'_rpe1_heading_rmse_deg']=m['rpe_1_frame']['heading_deg']['rmse']
    row['translation_improved']=metrics['change']['translation_m']['improved'];row['heading_improved']=metrics['change']['heading_deg']['improved']
    (out/'metrics_row.json').write_text(json.dumps(row,indent=2))
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--converted',type=Path,required=True);p.add_argument('--jobs',type=int,default=2);p.add_argument('--case',action='append',help='scene/case selection for adapter validation only')
    p.add_argument('--backend-config',type=Path,default=ROOT/'config/backend/simulation_multires_extended.txt');mode=p.add_mutually_exclusive_group();mode.add_argument('--samples',type=Path,help='Lossless Scan binary export root; defaults to converted samples or a fresh lossless export');mode.add_argument('--resample-endpoints',action='store_true',help='Explicit legacy experiment: reconstruct free samples at optimizer resolution');p.add_argument('--max-observations',type=int,default=100000000)
    a=p.parse_args();run=Path(os.environ['SDF_BA_RUN_ROOT']);inventory=json.loads((a.converted/'inventory.json').read_text())
    selected=[(a.converted/s['scene'],c) for s in inventory for c in s['cases'] if not a.case or s['scene']+'/'+c['name'] in a.case]
    if not selected:raise ValueError('No matching simulation cases')
    if not a.samples and not a.resample_endpoints:
        scenes={source.name for source,_ in selected}
        if all((a.converted/name/'samples.bin').is_file() for name in scenes):a.samples=a.converted
        else:
            from export_matlab_samples import export_all
            a.samples=export_all(a.converted,run/'exact_samples',scenes)
    (run/'protocol.json').write_text(json.dumps(dict(source=str(a.converted),cases=[s.name+'/'+c['name'] for s,c in selected],backend_config=str(a.backend_config) if a.backend_config else 'per-scene conversion profile',samples=str(a.samples) if a.samples else 'endpoint resampling',step_policy='legacy',all_beams=True,extra_frame_subsampling=False,N_derivative='unchanged paper approximation',notes='All supplied low-rate frames; GT is evaluation only; final accepted state, no GT-best step selection'),indent=2))
    shutil.copytree(ROOT/'tools',run/'tool_source',ignore=shutil.ignore_patterns('__pycache__'))
    rows=[]
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futures={pool.submit(run_case,s,c,run,a):s.name+'/'+c['name'] for s,c in selected}
        for future in as_completed(futures):
            name=futures[future]
            try:row=future.result()
            except Exception as exc:row=dict(scene=name.split('/')[0],case=name.split('/')[1],status='evaluation_or_runner_failed',error=repr(exc))
            rows.append(row);rows.sort(key=lambda r:(r['scene'],r['case']))
            (run/'metrics.json').write_text(json.dumps(rows,indent=2))
            fields=list(dict.fromkeys(k for r in rows for k in r))
            with (run/'metrics.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
            print(name,json.dumps(row),flush=True)
    if any(r['status']!='completed' for r in rows):raise RuntimeError('Incomplete runs retained in metrics.json')

if __name__=='__main__':main()
