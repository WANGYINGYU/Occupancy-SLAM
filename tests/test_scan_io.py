import sys
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from scan_io import Transforms, cloud_xyz, project_points, scan_match, se2


class ScanTests(unittest.TestCase):
    def test_tf_interpolation_and_extrinsic(self):
        g=Transforms(2)
        first,last,ext=np.eye(4),np.eye(4),np.eye(4)
        last[0,3]=2
        last[:2,:2]=se2([0,0,np.pi/2])[:2,:2]
        ext[0,3]=0.4
        g.add('odom','base',0,first)
        g.add('odom','base',2_000_000_000,last)
        g.add('base','laser',0,ext,True)
        g.finish()
        got=g.lookup('odom','laser',1_000_000_000)
        expected=np.array([1+0.4/np.sqrt(2),0.4/np.sqrt(2),0])
        np.testing.assert_allclose(got[:3,3],expected)
        np.testing.assert_allclose(got@g.lookup('laser','odom',1_000_000_000),np.eye(4),atol=1e-12)
        with self.assertRaises(ValueError):g.lookup('odom','laser',3_000_000_000)

    def test_missing_tf_is_not_identity(self):
        g=Transforms();g.finish()
        with self.assertRaises(ValueError):g.lookup('base','laser',100)

    def test_big_endian_organized_cloud_with_padding(self):
        raw=bytearray(2*40)
        dtype=np.dtype({'names':['x','y','z'],'formats':['>f4']*3,'offsets':[0,4,8],'itemsize':16})
        data=np.ndarray((2,2),dtype=dtype,buffer=raw,strides=(40,16))
        for i,k in enumerate(('x','y','z')):data[k]=np.arange(4).reshape(2,2)+10*i
        msg=NS(height=2,width=2,point_step=16,row_step=40,is_bigendian=True,data=np.frombuffer(raw,dtype=np.uint8),fields=[NS(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate(('x','y','z'))])
        np.testing.assert_array_equal(cloud_xyz(msg),np.column_stack([np.arange(4)+10*i for i in range(3)]))

    def test_projection_height_and_nearest_return(self):
        args=NS(min_height=-0.1,max_height=0.1,min_range=.1,max_range=20,bins=360)
        points=np.array([[2,0,0],[1,0,0],[.5,0,2],[np.nan,0,0],[0,3,0]])
        scan=project_points(points,1,'sensor',args)
        self.assertEqual(scan.ranges[180],1)
        self.assertEqual(scan.ranges[270],3)
        self.assertEqual(np.isfinite(scan.ranges).sum(),2)

    def test_scan_match_transform_direction(self):
        rng=np.random.default_rng(42)
        previous=rng.uniform(-3,3,(500,2))
        relative=se2([.08,-.04,.02])
        current=(previous-relative[:2,2])@relative[:2,:2]
        result,error=scan_match(previous,current,max_distance=.4)
        np.testing.assert_allclose(result,relative,atol=2e-5)
        self.assertLess(error,1e-4)


if __name__=='__main__':unittest.main()
