#!/usr/bin/env python3
"""Known-answer checks for Cartesian scans; all output stays in the run directory."""
import json,os,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from display_grid import integrate
from backend_config import load_parameters
import yaml

def main():
    run=Path(os.environ['SDF_BA_RUN_ROOT'])/'endpoint_checks';run.mkdir()
    config=load_parameters(ROOT/'config/backend/default.txt',dict(Scale=.2,MaxIter=0,NumBeam=20,MinAngle=-1.,MaxAngle=1.,MaxRange=15.,MinRange=0.,FixedGrid="true",Sizei=80,Sizej=80,OriginX=-3.013,OriginY=-5.017))
    (run/'config.txt').write_text(''.join(f'{k} {v}\n' for k,v in config.items()))
    # Keep this cross-language count check off exact cell boundaries: BLAS and
    # scalar arithmetic can differ by one ulp there, without a geometry error.
    poses=np.array([[0.,0.,.13],[.04,-.02,.17]])
    angle=np.linspace(-1.,1.,20); ranges=np.full((2,20),2.37)
    points=np.stack([ranges*np.cos(angle),ranges*np.sin(angle)],axis=-1)
    # Arbitrary beam perturbation: a Range/constant-angle conversion cannot retain this.
    points[1,:,1]+=np.linspace(-.003,.004,20)
    rows=np.column_stack([np.repeat(np.arange(2),20),points.reshape(-1,2)])
    np.savetxt(run/'Endpoints.txt',rows,fmt='%.17g');np.savetxt(run/'Pose.txt',poses,fmt='%.17g')
    command=[str(ROOT/'build/occupancy_slam_2d'),'--endpoints',str(run/'Endpoints.txt'),'--poses',str(run/'Pose.txt'),'--config',str(run/'config.txt'),'--output',str(run/'result')]
    subprocess.run(command,check=True,stdout=subprocess.DEVNULL)
    meta=yaml.safe_load((run/'result/initial/map.yaml').read_text()); m=np.loadtxt(run/'result/initial/occupancy.txt');counts=np.loadtxt(run/'result/initial/integration_counts.txt')
    actual,c=integrate(points,poses,config,dict(shape=m.shape,resolution=meta['resolution'],origin=meta['origin']))
    np.testing.assert_array_equal(c,counts);np.testing.assert_allclose(actual,m,atol=2e-14,rtol=0)
    for i in range(2):np.testing.assert_array_equal(points[i],rows[i*20:(i+1)*20,1:])
    rows[0,0]=.5;np.savetxt(run/'Bad.txt',rows)
    bad=command.copy();bad[2]=str(run/'Bad.txt');bad[-1]=str(run/'bad_result')
    result=subprocess.run(bad,capture_output=True,text=True)
    assert result.returncode!=0 and 'ordered integers' in result.stderr
    assert json.loads((run/'bad_result/run_status.json').read_text())['status']=='failed'
    print('Cartesian observation / display integration and invalid-index checks passed')

if __name__=='__main__':main()
