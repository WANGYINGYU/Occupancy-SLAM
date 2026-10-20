#!/usr/bin/env python3
import os,sys,tempfile,unittest
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from simulation_geometry import radius,reference_geometry,verify_feature_model
from evaluate_simulation_maps import contours,compare


class GeometryTests(unittest.TestCase):
    def test_fourier_coefficients(self):
        f=np.zeros(13);f[2]=3;f[3]=.5;f[8]=.25
        np.testing.assert_allclose(radius(f,np.array([0,np.pi/2,np.pi])),[3.5,3.25,2.5],atol=1e-15)

    def test_known_circle_and_empty_prediction(self):
        f=np.zeros((13,1));f[2]=.5
        angles=np.arange(360)*2*np.pi/360
        scans=np.stack([np.ones(360),.5*np.cos(angles),.5*np.sin(angles)])[None]
        geometry=dict(feature=f,rectF=np.empty((4,0)),borderF=np.array([-3,-3,6,6])[:,None])
        poses=np.zeros((1,3))
        reference,observed,report=reference_geometry(geometry,scans,poses)
        self.assertLess(verify_feature_model(geometry,scans,poses)['radial_max_m'],1e-15)
        self.assertGreater(report['observed_reference_points'],300)
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);x=np.arange(201)*.01-1;xx,yy=np.meshgrid(x,x)
            np.savetxt(folder/'occupancy.txt',.5-np.hypot(xx,yy));np.savetxt(folder/'normalization.txt',np.ones_like(xx))
            (folder/'map.yaml').write_text(yaml.safe_dump(dict(origin=[-1.,-1.,0.],resolution=.01)))
            points=contours(folder,.5,dict(yaw_rad=0.,translation_m=[0,0]))
        result=compare(points,reference,observed)
        self.assertLess(result['symmetric_capped_mae_m'],.005)
        self.assertEqual(result['f1_010'],1.)
        self.assertEqual(compare(np.empty((0,2)),reference,observed)['symmetric_capped_mae_m'],1.)


if __name__=='__main__':unittest.main()
