#!/usr/bin/env python3
"""Run the original MATLAB simulation functions on a supplied noisy initialization."""
import argparse,json,os,shutil,subprocess,tempfile,time
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,default=Path('/home/yingyuwang/Downloads/2D_Occupancy_SLAM_MATLAB'))
    ap.add_argument('--converted',type=Path,default=Path('/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_20261020_01/converted'))
    ap.add_argument('--matlab',type=Path,default=Path('/usr/local/bin/matlab'))
    ap.add_argument('--scene',required=True);ap.add_argument('--case',required=True)
    ap.add_argument('--iterations',type=int,default=20)
    args=ap.parse_args();root=Path(os.environ['SDF_BA_RUN_ROOT']);dest=root/'matlab_source';dest.mkdir()
    changes={}
    for path in args.source.glob('*.m'):
        source=path.read_text();text=source
        if path.name in ('FuncUpdateMapN.m','FuncUpdateSelectMapN.m'):text=text.replace('parfor ','for ')
        if path.name in ('FuncLeastSquares.m','FuncInitialSelectMap.m'):
            text='\n'.join('% headless I/O: '+line if line.strip().startswith(('figure(','figure (','imshow(','imwrite(')) else line for line in text.splitlines())+'\n'
        if path.name=='FuncDelta.m':text=text.replace('save(NameInfo,"II");','% Unused information output omitted.')
        if path.name=='FuncLeastSquares.m':text=text[:text.index('\nif Iter>0')]+'\nInfo=[]; % Unused information output omitted.\n'
        (dest/path.name).write_text(text)
        if text!=source:changes[path.name]='Display/information output or serial parfor only; mathematical operations retained'
    (root/'source_adjustments.json').write_text(json.dumps(changes,indent=2))
    here=Path(__file__).resolve().parents[1]
    shutil.copy2(here/'tests/run_matlab_reference.m',root/'run_matlab_reference.m');shutil.copy2(__file__,root/'runner.py')
    env=dict(os.environ,SIM_CONVERTED_ROOT=str(args.converted.resolve()),SIM_SCENE=args.scene,SIM_CASE=args.case,SIM_MAX_ITER=str(args.iterations),MATLAB_PREFDIR=str(root/'matlab_prefs'))
    short=tempfile.mkdtemp(prefix='m.',dir='/media/yingyuwang/Data/experiments/sdf_ba');env.update(TMPDIR=short,TMP=short,TEMP=short)
    cmd=[str(args.matlab),'-nojvm','-nodisplay','-batch',"run('"+str(root/'run_matlab_reference.m')+"')"]
    (root/'command.json').write_text(json.dumps(cmd,indent=2));start=time.monotonic();peak=swap=0;stopped=False
    with (root/'matlab.log').open('w') as log:
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while proc.poll() is None:
            try:
                fields=dict(line.split(':',1) for line in Path(f'/proc/{proc.pid}/status').read_text().splitlines() if ':' in line)
                peak=max(peak,int(fields.get('VmHWM','0 kB').split()[0]));swap=max(swap,int(fields.get('VmSwap','0 kB').split()[0]))
                if peak>68*1024*1024:
                    import signal
                    os.killpg(proc.pid,signal.SIGTERM);stopped=True
            except FileNotFoundError:pass
            time.sleep(1)
    (root/'runtime.json').write_text(json.dumps(dict(returncode=proc.returncode,wall_seconds=time.monotonic()-start,peak_rss_kib_sampled=peak,peak_swap_kib_sampled=swap,resource_stop=stopped),indent=2))
    print((root/'matlab.log').read_text()[-1800:]);raise SystemExit(proc.returncode)

if __name__=='__main__':main()
