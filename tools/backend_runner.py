"""Execute and describe a normalized 2D dataset, independent of input adapter."""
import json
import subprocess
from pathlib import Path
from export_maps import export_result

def run_backend(out, params, args, source):
    params.update(EvaluateGT='false',PosefromOdom='false',ModeOdom='true' if (out/'Odom.txt').exists() else 'false',ModeKeyFrame='false',ModeShowMap='false')
    (out/'backend_config.txt').write_text(''.join(f'{k} {v}\n' for k,v in params.items()))
    (out/'input_manifest.json').write_text(json.dumps(dict(schema_version=1,mode='2d',range_file='Range.txt',pose_file='Pose.txt',timestamp_file='timestamps.txt',odometry_file='Odom.txt' if (out/'Odom.txt').exists() else None,**source),indent=2))
    command=[str(args.backend),'--ranges',str(out/'Range.txt'),'--poses',str(out/'Pose.txt'),'--timestamps',str(out/'timestamps.txt'),'--config',str(out/'backend_config.txt'),'--output',str(out/'optimized'),'--step-policy',args.step_policy,'--max-observations',str(args.max_observations)]
    if (out/'Odom.txt').exists(): command+=['--odom',str(out/'Odom.txt')]
    (out/'backend_command.json').write_text(json.dumps(command,indent=2))
    status=dict(status='running',stage='optimization')
    (out/'run_status.json').write_text(json.dumps(status,indent=2))
    try:
        with (out/'backend.log').open('w') as log: subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        report=json.loads((out/'optimized/summary.json').read_text())
        status.update(stage='map_export',solver_stop_reason=report['stop_reason'])
        (out/'run_status.json').write_text(json.dumps(status,indent=2))
        export_result(out,args.display_resolution,out/'display_maps')
        status.update(status='completed',stage='complete')
    except BaseException as exc:
        status.update(status='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',error=str(exc))
        raise
    finally:
        (out/'run_status.json').write_text(json.dumps(status,indent=2))
