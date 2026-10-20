#!/usr/bin/env python3
"""Known geometry and native C++ equivalence checks for independent display grids."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from display_grid import integrate, shared_geometry, HIT_LOG_ODDS, FREE_LOG_ODDS
from rviz_viewer import parser, load_scene, load_grid


def synthetic():
    # Non-origin, rotated world poses and grid. Beam 0 faces the grid +x axis.
    config = dict(MinAngle=0, MaxAngle=np.pi/2, MinRange=.1, MaxRange=8)
    ranges = np.array([[2.4,0.]])
    poses = np.array([[4.537,10.237,np.pi/2]])
    for h in (1.,.5,.1,.05):
        shape = (int(4/h),int(4/h))
        geometry = dict(resolution=h,origin=[5.,10.,np.pi/2],shape=shape)
        raw,counts = integrate(ranges,poses,config,geometry)
        n = int(np.floor(2.4/h))
        # In grid coordinates, the sensor is (.237,.463), away from floating-point bin boundaries.
        expected_counts = np.zeros(shape,dtype=np.uint64)
        expected_raw = np.zeros(shape)
        for k in range(n):
            x = .237+2.4*(n-k)/n
            ix,iy = int(np.floor(x/h)),int(np.floor(.463/h))
            expected_counts[iy,ix] += 1
            expected_raw[iy,ix] += HIT_LOG_ODDS if k == 0 else FREE_LOG_ODDS
        np.testing.assert_array_equal(counts,expected_counts)
        np.testing.assert_allclose(raw,expected_raw,atol=1e-14,rtol=0)
    # Strict invalid-return filtering; no fabricated hits for zero/NaN/Inf/max range.
    config['MaxAngle'] = 1.
    raw,counts = integrate(np.array([[0.,np.nan,np.inf,8.,.1]]),poses,config,geometry)
    assert not counts.any() and not raw.any()
    native = dict(resolution=1.,origin=[5.,10.,np.pi/2],values=np.zeros((4,4)))
    for h in (0.,-1.,np.nan,np.inf,1e-9):
        try: shared_geometry(native,[],h)
        except ValueError: pass
        else: raise AssertionError(f'Invalid/oversized resolution accepted: {h}')


def real(pipeline):
    config = dict(line.split() for line in (pipeline/'backend_config.txt').read_text().splitlines() if line.strip())
    ranges = np.loadtxt(pipeline/'Range.txt')
    before = np.loadtxt(pipeline/'Pose.txt')
    after = np.loadtxt(pipeline/'optimized/poses.txt')
    for method, poses in (('initial',before),('reintegrated',after)):
        folder = pipeline/'optimized'/method
        grid = load_grid(folder)
        # At native resolution the new integrator must reproduce C++ values/counts.
        geometry = dict(resolution=grid['resolution'],origin=grid['origin'],shape=grid['values'].shape)
        raw,counts = integrate(ranges,poses,config,geometry)
        np.testing.assert_array_equal(counts,np.loadtxt(folder/'integration_counts.txt'))
        np.testing.assert_allclose(raw,np.loadtxt(folder/'occupancy.txt'),atol=1e-10,rtol=1e-12)
    records = []
    source_maps = None
    optimized_display = None
    for h in (.05,.1):
        args = parser().parse_args(['2d','--result',str(pipeline),'--display-resolution',str(h)])
        scene = load_scene(args)
        native = scene['maps']
        if source_maps is None:
            source_maps = {k:v['values'].copy() for k,v in native.items()}
            optimized_display = native['optimized']['display_grid']['values'].copy()
        else:
            for k,v in native.items(): np.testing.assert_array_equal(v['values'],source_maps[k])
            np.testing.assert_array_equal(native['optimized']['display_grid']['values'],optimized_display)
        initial, rebuilt = [native[m]['display_grid'] for m in ('initial','reintegrated')]
        assert initial['origin'] == rebuilt['origin']
        assert initial['values'].shape == rebuilt['values'].shape
        valid = (ranges > float(config['MinRange'])) & (ranges < float(config['MaxRange']))
        expected_samples = int(np.floor(ranges[valid]/h).sum())
        for display in (initial,rebuilt):
            assert display['resolution'] == h
            assert int(display['integration_counts'].sum()) == expected_samples
            assert np.all(display['values'][display['integration_counts'] == 0] == -1)
        assert native['optimized']['display_grid']['resolution'] == native['optimized']['resolution']
        assert not np.array_equal(initial['values'],rebuilt['values'])
        records.append(dict(display_resolution=h, raw_resolution=native['optimized']['resolution'],
                            shape=list(initial['values'].shape), origin=initial['origin'],
                            samples_per_method=expected_samples,
                            unchanged_native_maps=True, unchanged_joint_display=True,
                            methods={k:v['display_stats'] for k,v in native.items()}))
    assert records[0]['samples_per_method'] > records[1]['samples_per_method']
    return records


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pipeline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    synthetic()
    report = dict(known_geometry=True, cpp_native_equivalence=True, resolutions=real(args.pipeline))
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
