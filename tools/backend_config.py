"""Common original-format backend parameters and validation."""
import math
from project_paths import ROOT
def read_parameters(path):
    values = {}
    for number,line in enumerate(path.read_text().splitlines(),1):
        tokens = line.split('#',1)[0].split()
        if not tokens: continue
        if len(tokens) != 2 or tokens[0] in values:
            raise ValueError(f'Invalid/duplicate parameter at {path}:{number}')
        values[tokens[0]] = tokens[1]
    return values


def load_parameters(path, overrides=None):
    params=read_parameters(ROOT/'config.txt')
    supplied=read_parameters(path)
    known=set(params)|{'Sizei','Sizej','OriginX','OriginY','FixedGrid','InitialMapRound','NormalizationRound','WeightSmoothNSecond'}
    if supplied.keys()-known: raise ValueError(f'Unknown backend parameters: {sorted(supplied.keys()-known)}')
    params.update(supplied)
    params.update({k:str(v) for k,v in (overrides or {}).items() if v is not None})
    integers={'NumBeam','MaxIter','Sizei','Sizej','DownTime','DownRate','SelectKernelSize','KeyframeRate','SolverFirstMaxIter','SolverSecondMaxIter'}
    for key,value in params.items():
        if key.startswith('Mode') or key in ('EvaluateGT','PosefromOdom','FixedGrid','InitialMapRound','NormalizationRound'):
            if value not in ('true','false'): raise ValueError(f'{key} must be true or false')
        else:
            x=float(value)
            if not math.isfinite(x): raise ValueError(f'Nonfinite backend parameter: {key}')
            if key in integers and (x!=int(x) or x<0): raise ValueError(f'{key} must be a nonnegative integer')
    if int(params['NumBeam'])<2 or float(params['Scale'])<=0 or float(params['WeightO'])<0: raise ValueError('Invalid scan/grid/odometry parameters')
    if not 0<=float(params['MinRange'])<float(params['MaxRange']) or float(params['MinAngle'])>=float(params['MaxAngle']): raise ValueError('Invalid scan geometry')
    if int(params['SolverFirstMaxIter'])<1 or not 0<float(params['SolverFirstTolerance'])<1: raise ValueError('Invalid linear solver settings')
    if float(params['WeightSmoothN'])<=0 or float(params['MapSmoothingWeightFirst'])<=0: raise ValueError('Smoothing weights must be positive')
    if params.get('ModeMulti')=='true':
        if int(params['DownRate'])<1 or int(params['SolverSecondMaxIter'])<1 or not 0<float(params['SolverSecondTolerance'])<1: raise ValueError('Invalid multiresolution solver settings')
        if float(params['MapSmoothingWeightSecond'])<=0 or float(params.get('WeightSmoothNSecond',1))<=0: raise ValueError('Fine smoothing weights must be positive')
    return params
