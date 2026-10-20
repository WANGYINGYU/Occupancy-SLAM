"""Independent ground-truth geometry stored in the MATLAB simulator files."""
import numpy as np
from scipy.spatial import cKDTree


def radius(feature, theta):
    harmonics=(np.arange(5)+1)[:,None]*np.asarray(theta).ravel()[None,:]
    return feature[2]+feature[3:8]@np.cos(harmonics)+feature[8:13]@np.sin(harmonics)


def world_hits(scans, poses):
    local=scans[:,1:,:].transpose(0,2,1)
    c,s=np.cos(poses[:,2,None]),np.sin(poses[:,2,None])
    return np.stack([c*local[:,:,0]-s*local[:,:,1]+poses[:,0,None],
                     s*local[:,:,0]+c*local[:,:,1]+poses[:,1,None]],axis=2).reshape(-1,2)


def inside_objects(points, feature, rectangles, skip=None, margin=0.):
    inside=np.zeros(len(points),dtype=bool)
    for i in range(feature.shape[1]):
        if skip==('feature',i):continue
        d=points-feature[:2,i];theta=np.arctan2(d[:,1],d[:,0])
        inside |= np.linalg.norm(d,axis=1)<radius(feature[:,i],theta)-margin
    for i,rect in enumerate(rectangles.T):
        if skip==('rectangle',i):continue
        x,y,w,h=rect
        inside |= ((points[:,0]>x+margin)&(points[:,0]<x+w-margin)&(points[:,1]>y+margin)&(points[:,1]<y+h-margin))
    return inside


def polyline_samples(points, spacing):
    samples=[]
    for a,b in zip(points[:-1],points[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/spacing)))
        samples.append(a+np.arange(n)[:,None]/n*(b-a))
    return np.concatenate(samples) if samples else np.empty((0,2))


def rectangle_points(rect,spacing):
    x,y,w,h=rect
    return polyline_samples(np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h],[x,y]]),spacing)


def reference_geometry(geometry, raw_scans, poses, spacing=.005):
    feature=geometry['feature'];rect=geometry['rectF'];border=geometry['borderF'].ravel()
    parts=[]; max_chord=0.
    for i in range(feature.shape[1]):
        f=feature[:,i]
        # Bound the curve speed so both point spacing and KD error remain controlled.
        speed_bound=abs(f[2])+sum((k+2)*(abs(f[3+k])+abs(f[8+k])) for k in range(5))
        n=max(256,int(np.ceil(2*np.pi*speed_bound/spacing)))
        theta=np.arange(n)*2*np.pi/n;rho=radius(f,theta)
        if np.min(rho)<=0:raise ValueError('Nonpositive simulator radial boundary')
        pts=f[:2]+np.column_stack([rho*np.cos(theta),rho*np.sin(theta)])
        max_chord=max(max_chord,float(np.linalg.norm(pts-np.roll(pts,1,axis=0),axis=1).max()))
        parts.append(pts[~inside_objects(pts,feature,rect,('feature',i),1e-9)])
    for i,r in enumerate(rect.T):
        pts=rectangle_points(r,spacing)
        parts.append(pts[~inside_objects(pts,feature,rect,('rectangle',i),1e-9)])
    pts=rectangle_points(border,spacing)
    parts.append(pts[~inside_objects(pts,feature,rect,margin=1e-9)])
    reference=np.concatenate(parts)
    x,y,w,h=border
    reference=reference[(reference[:,0]>=x-1e-9)&(reference[:,0]<=x+w+1e-9)&(reference[:,1]>=y-1e-9)&(reference[:,1]<=y+h+1e-9)]
    # Resample geometry approximately uniformly in arc length by spatial binning.
    # Keep actual geometry points, not voxel centroids.
    _,idx=np.unique(np.floor(reference/spacing).astype(np.int64),axis=0,return_index=True)
    reference=reference[np.sort(idx)]
    tree=cKDTree(reference); world=world_hits(raw_scans,poses)
    raw_distance,nearest=tree.query(world,workers=1)
    projected=reference[np.unique(nearest)]
    observed=cKDTree(projected).query(reference,workers=1)[0] <= .10
    return reference,observed,dict(reference_points=len(reference),observed_reference_points=int(observed.sum()),
        geometry_spacing_m=spacing,max_curve_chord_m=max_chord,observed_support_radius_m=.10,
        raw_hit_to_geometry_rmse_m=float(np.sqrt(np.mean(raw_distance**2))),raw_hit_to_geometry_p99_m=float(np.percentile(raw_distance,99)),
        raw_hit_to_geometry_max_m=float(np.max(raw_distance)))


def verify_feature_model(geometry,scans,poses):
    points=world_hits(scans,poses);labels=scans[:,0,:].ravel();feature=geometry['feature'];errors=[]
    for i in range(feature.shape[1]):
        p=points[labels==i+1]-feature[:2,i]
        errors.extend(np.linalg.norm(p,axis=1)-radius(feature[:,i],np.arctan2(p[:,1],p[:,0])))
    errors=np.asarray(errors)
    return dict(feature_hit_count=len(errors),radial_rmse_m=float(np.sqrt(np.mean(errors**2))),radial_max_m=float(np.max(np.abs(errors))),
        formula='rho(theta)=feature[2]+sum(feature[3:8]*cos(k theta)+feature[8:13]*sin(k theta)), k=1..5; first two rows are centre; rectangles [x,y,width,height]')
