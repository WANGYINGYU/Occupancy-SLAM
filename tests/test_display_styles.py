#!/usr/bin/env python3
"""Known-coordinate support and display classification checks; source files stay unchanged."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from rviz_viewer import observation_support, classified_grid, height_rgb

config = dict(MinAngle=0., MaxAngle=np.pi/2, MinRange=.01, MaxRange=10.)
grid = dict(resolution=1., origin=[0.,0.,0.], values=np.zeros((6,6)))
scan = np.array([[2.,0.]])
for yaw, expected_floor, expected_bilinear in [
    (0., {(1,2),(1,3)}, {(y,x) for y in (1,2) for x in (2,3,4)}),
    (np.pi/2, {(2,1),(3,1)}, {(y,x) for y in (2,3,4) for x in (1,2)}),
]:
    pose = np.array([[1.25,1.25,yaw]])
    assert set(zip(*np.where(observation_support(scan, pose, config, grid)))) == expected_floor
    assert set(zip(*np.where(observation_support(scan, pose, config, grid, True)))) == expected_bilinear
# Exactly on lattice: zero-weight neighbouring vertices are not observations.
assert set(zip(*np.where(observation_support(scan, np.array([[1.,1.,0.]]), config, grid, True)))) == {(1,2),(1,3)}
# Rotated map origin, not a second transform of the saved world point cloud.
grid['origin'] = [5.,0.,np.pi/2]
assert set(zip(*np.where(observation_support(scan, np.array([[1.25,1.25,0.]]), config, grid)))) == {(1,1),(2,1)}
# Unseen cells remain unknown even if the optimized field extrapolates strongly.
raw = np.array([[-5.,0.,5.,-100.,100.]])
observed = np.array([[True,True,True,False,False]])
np.testing.assert_array_equal(classified_grid(raw,observed), [[0,-1,100,-1,-1]])
np.testing.assert_array_equal(height_rgb(np.array([-5.,0.,1.,5.]),0.,1.), [0x245DAA,0x245DAA,0xCD3E3E,0xCD3E3E])
print('PASS: floor/bilinear support, zero weights, rotated grid, unknown coverage and classification')
