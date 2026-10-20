#!/usr/bin/env python3
"""Losslessly export original Scan.xy/Odd cells for the C++ --samples adapter."""
import argparse,json,os,struct,shutil
from pathlib import Path
import numpy as np
from matlab_simulation import scan_cells

def export_scene(source,target,frames):
    source,target=Path(source),Path(target);counts=[]
    with target.open('xb') as f:
        f.write(struct.pack('<Q',frames))
        for xy,labels in scan_cells(source):
            if xy.shape!=(len(labels),2) or not np.isfinite(xy).all() or not np.isfinite(labels).all():raise ValueError('Invalid Scan samples')
            f.write(struct.pack('<Q',len(labels)));f.write(np.asarray(xy,dtype='<f8').tobytes());f.write(np.asarray(labels,dtype='<f8').tobytes());counts.append(len(labels))
    if len(counts)!=frames:raise ValueError('Frame count mismatch')
    return dict(source=str(source),frames=len(counts),samples=sum(counts),per_frame=counts,format='little-endian uint64 frame count; each frame: uint64 n; interleaved XY float64; labels float64',resampling=False)

def export_all(converted,root,scenes=None):
    receipts=[]
    for scene in json.loads((converted/'inventory.json').read_text()):
        if scenes is not None and scene['scene'] not in scenes:continue
        receipt=json.loads((converted/scene['scene']/'conversion.json').read_text());out=root/scene['scene'];out.mkdir(parents=True)
        record=export_scene(receipt['scan_file'],out/'samples.bin',scene['frames']);record['scene']=scene['scene']
        (out/'receipt.json').write_text(json.dumps(record,indent=2));receipts.append(record);print(scene['scene'],record['samples'],'exact samples',flush=True)
    (root/'inventory.json').write_text(json.dumps(receipts,indent=2))
    return root

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--converted',type=Path,required=True);a=p.parse_args()
    root=Path(os.environ['OCCUPANCY_RUN_DIR']);shutil.copy2(__file__,root/'export_source.py');export_all(a.converted,root)
if __name__=='__main__':main()
