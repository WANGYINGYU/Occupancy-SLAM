#!/usr/bin/env python3
"""Rebuild and cache independent-resolution display maps without ROS."""
import argparse,json,math,os,fcntl,tempfile,shutil
from pathlib import Path
from types import SimpleNamespace
from result_scene import load_scene,save_display_maps
from result_layout import ResultLayout

CACHE_VERSION=1
DATA=Path(os.environ.get('OCCUPANCY_DATA_ROOT','/media/yingyuwang/Data')).resolve()

def source_signature(result):
    """Invalidate on source/renderer edits, without hashing large scan payloads."""
    result=Path(result).resolve();layout=ResultLayout.read(result/'optimized')
    paths=[result/name for name in ('Pose.txt','timestamps.txt','backend_config.txt','Endpoints.npz','Range.txt')]
    paths.append(layout.root/'map_outputs.json')
    for folder in (layout.initial,layout.joint,layout.reintegrated):
        paths.extend(folder/name for name in ('poses.txt','occupancy.txt','normalization.txt','map.yaml'))
    paths.extend(Path(__file__).with_name(name) for name in
                 ('result_scene.py','display_grid.py','result_io.py','backend_config.py','geometry.py'))
    files=[]
    for path in paths:
        stat=path.stat() if path.is_file() else None
        files.append(dict(path=str(path),size=stat.st_size if stat else None,
                          mtime_ns=stat.st_mtime_ns if stat else None))
    return dict(version=CACHE_VERSION,result=str(result),files=files)

def export_result(result,resolution,output):
    result=Path(result).resolve();output=Path(output).resolve()
    if not math.isfinite(resolution) or resolution<=0:raise ValueError('Display resolution must be positive')
    if output.exists() and any(output.iterdir()):raise ValueError(f'Export directory must be empty: {output}')
    signature=source_signature(result)
    args=SimpleNamespace(mode='2d',result=result,initial=None,display_resolution=resolution,max_points=0,occupancy_threshold=.6,height_range=None,allow_running=True)
    scene=load_scene(args)
    output.mkdir(parents=True,exist_ok=True);save_display_maps(scene,output,nested=False)
    if source_signature(result)!=signature:raise RuntimeError('Display inputs changed during export; retry after the result is complete')
    manifest=dict(schema_version=1,result=str(result),resolution=resolution,integration=scene['display_integration'],methods={k:v['display_stats'] for k,v in scene['maps'].items()},source_signature=signature)
    manifest['output_sizes']={str(p.relative_to(output)):p.stat().st_size for p in output.rglob('*') if p.is_file()}
    (output/'display_manifest.json').write_text(json.dumps(manifest,indent=2))
    return output

def valid_cache(folder,resolution,signature):
    try:
        manifest=json.loads((folder/'display_manifest.json').read_text())
        if manifest.get('source_signature')!=signature or manifest['resolution']!=resolution:return False
        sizes=manifest['output_sizes']
        for method in manifest['methods']:
            for name in ('map.yaml','map.pgm','grid.npz'):
                relative=method+'/'+name
                if (folder/relative).stat().st_size!=sizes[relative]:return False
        return True
    except (OSError,ValueError,KeyError,TypeError):return False

def prepare_display(result,resolution,viewer_output):
    """Reuse a complete map across viewer runs; keep all cache data on the data disk.

    Immutable generations let an existing viewer continue reading while another
    builds a new resolution/source state. The per-entry lock serializes writers.
    viewer_output remains in the API for existing callers; it no longer owns the cache.
    """
    if not math.isfinite(resolution) or resolution<=0:raise ValueError('Display resolution must be positive')
    result=Path(result).resolve();signature=source_signature(result)
    candidate=result/'display_maps'
    if valid_cache(candidate,resolution,signature):return candidate
    storage=Path(os.environ.get('SDF_BA_STORAGE_ROOT',str(DATA/'experiments/sdf_ba'))).resolve()
    if not DATA.is_mount() or not storage.is_relative_to((DATA/'experiments/sdf_ba').resolve()):
        raise RuntimeError('Display cache requires the experiment data disk')
    if os.statvfs(storage).f_flag & os.ST_RDONLY:raise RuntimeError('Experiment data disk is read-only')
    # Preserve path components as the cache key; no content hash or source-tree writes.
    parent=storage/'occupancy_display_cache'/result.relative_to(result.anchor)/('resolution_'+float(resolution).hex())
    parent.mkdir(parents=True,exist_ok=True)
    with (parent/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        signature=source_signature(result)
        pointer=parent/'current.json'
        try:
            name=json.loads(pointer.read_text())['generation']
            if Path(name).name!=name:raise ValueError('Invalid cache generation')
            cached=parent/name
            if valid_cache(cached,resolution,signature):return cached
        except (OSError,ValueError,KeyError,TypeError):pass
        pending=Path(tempfile.mkdtemp(prefix='.pending_',dir=parent))
        try:
            export_result(result,resolution,pending)
            generation=parent/('maps_'+pending.name.removeprefix('.pending_'))
            pending.rename(generation)
            temporary=parent/('current_'+generation.name+'.json')
            temporary.write_text(json.dumps(dict(generation=generation.name)))
            temporary.replace(pointer)
            return generation
        finally:
            if pending.exists():shutil.rmtree(pending)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',type=Path,required=True);p.add_argument('--resolution',type=float,default=.05);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(export_result(a.result,a.resolution,a.output))
if __name__=='__main__':main()
