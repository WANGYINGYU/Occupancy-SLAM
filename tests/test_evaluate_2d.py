#!/usr/bin/env python3
"""Independent known-answer checks for pose evaluation and correspondence."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from evaluate_2d_map import cast_rays, sample_log_odds
from evaluate_2d import (apply_alignment, first_pose_alignment, fitted_alignment,
                         pose_metrics, match_ground_truth, relative_motion)


class EvaluationTests(unittest.TestCase):
    def test_rigid_gauge_and_scale(self):
        truth = np.array([[0.,0.,0.], [1.,0.,.2], [1.,1.,.4], [0.,1.,.6]])
        estimate = apply_alignment(truth, .8, np.array([4.,-2.]))
        aligned = apply_alignment(estimate, *first_pose_alignment(estimate, truth))
        np.testing.assert_allclose(aligned, truth, atol=2e-15)
        scaled = truth.copy(); scaled[:, :2] *= 2
        fitted = apply_alignment(scaled, *fitted_alignment(scaled, truth))
        self.assertGreater(pose_metrics(fitted, truth)[0]['translation_m']['rmse'], .7)

    def test_known_translation_and_wrapped_yaw(self):
        truth = np.array([[0.,0.,0.], [0.,0.,np.pi-.01], [0.,0.,0.]])
        estimate = truth.copy(); estimate[1,:2] = [3,4]; estimate[1,2] = -np.pi+.01
        report, _, heading = pose_metrics(estimate, truth)
        self.assertAlmostEqual(report['translation_m']['rmse'], np.sqrt(25/2))
        self.assertAlmostEqual(report['coordinate_rmse_m'], 2.5)
        self.assertAlmostEqual(report['matlab_axis_mean_rmse_m'], 7/(2*np.sqrt(2)))
        self.assertAlmostEqual(heading[1], np.degrees(.02))

    def test_relative_motion_uses_local_frame(self):
        poses = np.array([[0.,0.,np.pi/2], [0.,1.,np.pi]])
        np.testing.assert_allclose(relative_motion(poses,1), [[1,0,np.pi/2]], atol=1e-15)

    def test_native_field_interpolation_and_missing_ray(self):
        # Field is negative until x=2, then positive. No measured samples required.
        field=np.tile(np.arange(5.)-2,(3,1)); n=np.ones_like(field)
        np.testing.assert_allclose(sample_log_odds(field,n,np.array([[2.5,.5]]),[0,0],1),[.5])
        rays=cast_rays(field,n,np.array([[0.,1.],[0.,1.]]),np.array([[1.,0.],[-1.,0.]]),
                       [0,0],1,max_range=4,step=.025,thresholds=(.5,))[.5]
        self.assertAlmostEqual(rays[0],2.025)
        self.assertTrue(np.isnan(rays[1]))

    def test_correspondence_never_drops_results(self):
        gt = np.column_stack([np.arange(4.)+1000,np.zeros((4,3))])
        _, rows = match_ground_truth(gt, 2, np.array([1001.,1003.]))
        np.testing.assert_array_equal(rows, [1,3])
        with self.assertRaises(ValueError): match_ground_truth(gt, 2, np.array([1001.,1004.]))
        with self.assertRaises(ValueError): match_ground_truth(gt[:,1:], 2)
        with self.assertRaises(ValueError): match_ground_truth(gt[:,1:], 2, indices=np.array([0.,1.5]))


if __name__ == '__main__': unittest.main()
