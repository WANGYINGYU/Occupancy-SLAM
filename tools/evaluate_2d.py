#!/usr/bin/env python3
"""Evaluate matched 2D poses against independent GT; no optimizer or ROS imports."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from result_io import read_matrix
from result_layout import ResultLayout


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def apply_alignment(poses, angle, translation):
    result = poses.copy()
    result[:, :2] = poses[:, :2] @ rotation(angle).T + translation
    result[:, 2] = wrap(poses[:, 2] + angle)
    return result


def first_pose_alignment(estimate, truth):
    angle = float(wrap(truth[0, 2] - estimate[0, 2]))
    translation = truth[0, :2] - rotation(angle) @ estimate[0, :2]
    return angle, translation


def fitted_alignment(estimate, truth):
    x, y = estimate[:, :2], truth[:, :2]
    xc, yc = x - x.mean(axis=0), y - y.mean(axis=0)
    if min(np.linalg.norm(xc), np.linalg.norm(yc)) < 1e-12:
        return None
    u, _, vt = np.linalg.svd(xc.T @ yc)
    correction = np.diag([1., np.linalg.det(vt.T @ u.T)])
    r = vt.T @ correction @ u.T
    return float(np.arctan2(r[1, 0], r[0, 0])), y.mean(axis=0) - r @ x.mean(axis=0)


def statistics(values):
    values = np.asarray(values)
    return dict(rmse=float(np.sqrt(np.mean(values ** 2))),
                mean=float(np.mean(np.abs(values))),
                median=float(np.median(np.abs(values))),
                p95=float(np.percentile(np.abs(values), 95)),
                maximum=float(np.max(np.abs(values))))


def relative_motion(poses, delta):
    displacement = poses[delta:, :2] - poses[:-delta, :2]
    c, s = np.cos(poses[:-delta, 2]), np.sin(poses[:-delta, 2])
    return np.column_stack((c*displacement[:, 0]+s*displacement[:, 1],
                            -s*displacement[:, 0]+c*displacement[:, 1],
                            wrap(poses[delta:, 2]-poses[:-delta, 2])))


def pose_metrics(estimate, truth):
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 3 or len(estimate) < 2:
        raise ValueError('Need at least two finite, matched N x 3 poses')
    if not np.isfinite(estimate).all() or not np.isfinite(truth).all():
        raise ValueError('Nonfinite pose')
    # Exclude fixed first pose from absolute metrics; preserve original rows in CSV.
    translation = np.linalg.norm(estimate[:, :2] - truth[:, :2], axis=1)
    heading = np.degrees(wrap(estimate[:, 2] - truth[:, 2]))
    result = dict(evaluated_absolute_frames=len(estimate)-1,
                  translation_m=statistics(translation[1:]),
                  heading_deg=statistics(heading[1:]),
                  coordinate_rmse_m=float(np.sqrt(np.mean((estimate[1:, :2]-truth[1:, :2])**2))),
                  matlab_axis_mean_rmse_m=float(np.sqrt(np.mean((estimate[1:, :2]-truth[1:, :2])**2,axis=0)).mean()))
    for delta in (1, 10):
        if len(estimate) <= delta:
            continue
        a, b = relative_motion(estimate, delta), relative_motion(truth, delta)
        result[f'rpe_{delta}_frame'] = dict(pairs=len(a),
            translation_m=statistics(np.linalg.norm(a[:, :2]-b[:, :2], axis=1)),
            heading_deg=statistics(np.degrees(wrap(a[:, 2]-b[:, 2]))))
    return result, translation, heading


def match_ground_truth(gt, frame_count, timestamps=None, indices=None):
    if gt.shape[1] not in (3, 4):
        raise ValueError('GT must be x y yaw, or timestamp x y yaw (metres/radians)')
    if indices is not None:
        if gt.shape[1] != 3:
            raise ValueError('Use indices with 3-column GT, or timestamped GT without indices')
        if indices.shape != (frame_count,) or np.any(indices != np.floor(indices)) or np.any(np.diff(indices) <= 0):
            raise ValueError('Indices must be strictly increasing integers matching every result frame')
        indices = indices.astype(int)
        if indices[0] < 0 or indices[-1] >= len(gt):
            raise ValueError('GT index out of range')
        return gt[indices], indices
    if gt.shape[1] == 3:
        if len(gt) != frame_count:
            raise ValueError('GT row count differs; supply --indices or timestamped GT')
        return gt, np.arange(frame_count)
    if np.any(np.diff(gt[:, 0]) <= 0) or timestamps is None or len(timestamps) != frame_count:
        raise ValueError('Need increasing GT times and one timestamp per result frame')
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError('Result timestamps must strictly increase')
    right = np.searchsorted(gt[:, 0], timestamps).clip(0, len(gt)-1)
    left = (right-1).clip(0, len(gt)-1)
    rows = np.where(abs(gt[right, 0]-timestamps) < abs(gt[left, 0]-timestamps), right, left)
    if np.any(abs(gt[rows, 0]-timestamps) > 1e-6) or len(np.unique(rows)) != frame_count:
        raise ValueError('Every result timestamp must uniquely match GT within 1 microsecond; no silent drops')
    return gt[rows, 1:], rows


def evaluate(result, ground_truth, output, indices=None):
    result, ground_truth, output = Path(result), Path(ground_truth), Path(output)
    native = result/'optimized' if (result/'optimized/map_outputs.json').exists() else result
    for folder in {result, native}:
        status = folder/'run_status.json'
        if status.exists() and json.loads(status.read_text()).get('status') != 'completed':
            raise ValueError(f'Result has not completed: {folder}')
    layout = ResultLayout.read(native)
    poses = {key: read_matrix(getattr(layout, key)/'poses.txt', 3)
             for key in ('initial', 'joint', 'reintegrated')}
    count = len(poses['initial'])
    if any(len(p) != count for p in poses.values()):
        raise ValueError('Result methods have different frame counts')
    if not np.allclose(poses['joint'], poses['reintegrated'], rtol=0, atol=1e-12):
        raise ValueError('Joint and reintegrated poses differ')
    time_file = result/'timestamps.txt'
    times = read_matrix(time_file, 1).ravel() if time_file.exists() else None
    index_values = read_matrix(indices, 1).ravel() if indices else None
    gt = read_matrix(ground_truth)
    truth, rows = match_ground_truth(gt, count, times, index_values)
    angle, translation = first_pose_alignment(poses['initial'], truth)
    report = dict(result=str(result.resolve()), ground_truth=str(ground_truth.resolve()),
        input_gt_frames=len(gt), matched_frames=count, matched_gt_rows=rows.tolist(),
        protocol='One rigid SE2 transform from initial first pose, shared by all methods; no scale; first pose excluded from absolute metrics; wrapped angle errors; RPE on matched frame sequence',
        shared_alignment=dict(yaw_rad=angle, translation_m=translation.tolist()), methods={},
        map_accuracy_evaluated=False)
    output.mkdir(parents=True, exist_ok=False)
    for method, p in poses.items():
        aligned = apply_alignment(p, angle, translation)
        metrics, position_error, heading_error = pose_metrics(aligned, truth)
        fit = fitted_alignment(p, truth)
        if fit is not None:
            metrics['secondary_independently_fitted_se2'] = pose_metrics(apply_alignment(p, *fit), truth)[0]
        report['methods'][method] = metrics
        np.savetxt(output/(method+'_aligned_poses.txt'), aligned, fmt='%.17g')
        with (output/(method+'_errors.csv')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['result_row', 'gt_row', 'included_absolute', 'translation_m', 'heading_deg'])
            writer.writerows((i, int(rows[i]), int(i != 0), position_error[i], heading_error[i]) for i in range(count))
    initial = report['methods']['initial']
    final = report['methods']['joint']
    report['change'] = {name: dict(initial=initial[name]['rmse'], final=final[name]['rmse'],
        final_minus_initial=final[name]['rmse']-initial[name]['rmse'],
        improved=final[name]['rmse'] < initial[name]['rmse']) for name in ('translation_m', 'heading_deg')}
    (output/'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--ground-truth', type=Path, required=True)
    parser.add_argument('--indices', type=Path, help='Explicit GT row indices for selected TXT scans')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = evaluate(args.result, args.ground_truth, args.output, args.indices)
    print(json.dumps(report['change'], indent=2))


if __name__ == '__main__':
    main()
