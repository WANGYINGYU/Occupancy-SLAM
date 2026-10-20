"""Fixed GT-ray evaluation of native M/N fields, independent of display maps."""
from pathlib import Path
import numpy as np
import yaml
from evaluate_2d import rotation, statistics


def sample_log_odds(field, normalization, xy, origin, resolution):
    lattice = (xy-np.asarray(origin))/resolution
    lower = np.floor(lattice).astype(int)
    valid = ((lower[:,0]>=0) & (lower[:,1]>=0) &
             (lower[:,0]<field.shape[1]-1) & (lower[:,1]<field.shape[0]-1))
    values = np.full(len(xy), np.nan)
    x, y = lower[valid].T; dx, dy = (lattice[valid]-lower[valid]).T
    weights = [(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy]
    m = sum(w*field[y+oy,x+ox] for w,(ox,oy) in zip(weights,[(0,0),(1,0),(0,1),(1,1)]))
    n = sum(w*normalization[y+oy,x+ox] for w,(ox,oy) in zip(weights,[(0,0),(1,0),(0,1),(1,1)]))
    values[valid] = np.divide(m,n,out=np.full_like(m,np.nan),where=n>1e-9)
    return values


def cast_rays(field, normalization, origins, directions, map_origin, resolution,
              alignment_yaw=0., alignment_translation=(0.,0.), max_range=15., step=.025,
              thresholds=(.5,.65)):
    # Transform fixed GT rays to map coordinates; never use estimated query poses.
    inverse_rotation = rotation(alignment_yaw)
    origins = (origins-np.asarray(alignment_translation)) @ inverse_rotation
    directions = directions @ inverse_rotation
    distances = np.arange(.1,max_range+step/2,step)
    result = {p:np.full(len(origins),np.nan) for p in thresholds}
    for start in range(0,len(origins),128):
        end = min(start+128,len(origins))
        points = origins[start:end,None,:]+directions[start:end,None,:]*distances[None,:,None]
        log_odds = sample_log_odds(field,normalization,points.reshape(-1,2),map_origin,resolution).reshape(end-start,-1)
        for p in thresholds:
            # Strict threshold makes zero-evidence p=0.5 unknown, not occupied.
            occupied = log_odds > np.log(p/(1-p))+1e-12
            exists = occupied.any(axis=1)
            first = occupied.argmax(axis=1)
            hit = np.full(end-start,np.nan); hit[exists] = distances[first[exists]]
            result[p][start:end] = hit
    return result


def room_rays(poses):
    # Offset beams from the observations: 0.5, 2.5, ... degrees, full circle.
    angles = np.deg2rad(np.arange(180)*2+.5)-np.pi
    world_angles = (poses[:,2,None]+angles).ravel()
    directions = np.column_stack([np.cos(world_angles),np.sin(world_angles)])
    origins = np.repeat(poses[:,:2],180,axis=0)
    dx,dy = directions.T; x,y = origins.T
    with np.errstate(divide='ignore'):
        truth = np.minimum(np.where(dx>0,(5-x)/dx,(-4-x)/dx),
                           np.where(dy>0,(4-y)/dy,(-3-y)/dy))
    return origins,directions,truth


def evaluate_room_maps(native, truth_poses, shared_alignment):
    native=Path(native); origins,directions,truth=room_rays(truth_poses)
    report=dict(protocol='Fixed analytic rectangle rays at GT sensor poses; shared initial first-pose SE2 gauge; bilinear(M)/bilinear(N); raw native field, no display classification or map ICP',
        rays=len(truth),step_m=.025,max_range_m=15.,missing_penalty_m=15.,
        thresholds=[.5,.65],beam_offset_deg=.5,methods={})
    arrays=dict(gt_range=truth)
    for method in ('initial','joint','reintegrated'):
        folder=native/method
        field=np.loadtxt(folder/'occupancy.txt'); normalization=np.loadtxt(folder/'normalization.txt')
        meta=yaml.safe_load((folder/'map.yaml').read_text())
        if float(meta['origin'][2]) != 0: raise ValueError('Expected axis-aligned native map')
        predicted=cast_rays(field,normalization,origins,directions,meta['origin'][:2],meta['resolution'],
            shared_alignment['yaw_rad'],shared_alignment['translation_m'])
        report['methods'][method]={}
        for threshold,ranges in predicted.items():
            good=np.isfinite(ranges); error=np.abs(ranges[good]-truth[good])
            report['methods'][method][str(threshold)]=dict(
                hit_rays=int(good.sum()),coverage=float(good.mean()),
                hit_error_m=statistics(error) if len(error) else None,
                penalized_mae_m=float((np.minimum(error,15.).sum()+(~good).sum()*15.)/len(truth)))
            arrays[method+'_'+str(threshold)]=ranges
    return report,arrays
