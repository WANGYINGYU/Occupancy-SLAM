function [DeltaP,DeltaM,MeanDelta,MeanDeltaPose] = Func6DoFDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH,Param,MapVarId,RegEHOffset)

if nargin < 9
    MapVarId = [];
end
if nargin < 10
    RegEHOffset = [];
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

LambdaO = Param.LambdaO;
InfMatO = Param.InfMatO;

IO = FuncGet3DIO(ErrorO,InfMatO(1),InfMatO(2),InfMatO(3),InfMatO(4),InfMatO(5),InfMatO(6));

JP = JP(:,7:end);
JO = JO(:,7:end);

% Keep pose-state blocks aligned even if one term has no constraints on
% trailing poses (e.g., empty scan residuals near sequence tail).
nPoseCols = max(size(JP,2), size(JO,2));
if size(JP,2) < nPoseCols
    JP(:,end+1:nPoseCols) = 0;
end
if size(JO,2) < nPoseCols
    JO(:,end+1:nPoseCols) = 0;
end

U = JP'*JP+LambdaO*(JO'*IO*JO);
V = JM'*JM;
W = JP'*JM;

ErrorS = sparse(ErrorS);
ErrorO = sparse(ErrorO);
EP = -JP'*ErrorS - LambdaO*JO'*IO*ErrorO;
EM = -JM'*ErrorS;

clearvars JM JO

XH0 = reshape(permute(Map.Grid, [2, 1, 3]), Size_i * Size_j * Size_h, 1);
if ~isempty(MapVarId)
    XH0 = XH0(MapVarId);
elseif size(HH,1) ~= numel(XH0)
    XH0 = zeros(size(HH,1),1);
end

EH = -HH*XH0;
if ~isempty(RegEHOffset)
    EH = EH + RegEHOffset;
end

EM = EM+EH;

II = [U,W;
      W',V+HH];

OptimizerType = 'GN';
if isfield(Param,'OptimizerType') && ~isempty(Param.OptimizerType)
    OptimizerType = upper(Param.OptimizerType);
end

if strcmp(OptimizerType,'LM')
    LMDamping = 1e-3;
    if isfield(Param,'LMDamping') && Param.LMDamping > 0
        LMDamping = Param.LMDamping;
    end
    DiagII = abs(spdiags(II,0));
    DiagII(DiagII < 1e-9) = 1;
    II = II + LMDamping * spdiags(DiagII, 0, length(DiagII), length(DiagII));
elseif ~strcmp(OptimizerType,'GN')
    error('Unknown OptimizerType: %s. Use ''GN'' or ''LM''.', OptimizerType);
end

clearvars U W V;
  
EE = [EP;EM];
UsePCGSolver = isfield(Param,'UsePCGSolver') && Param.UsePCGSolver;
if UsePCGSolver
    persistent PCGWarmStart PCGWarmSize

    nSys = size(II,1);
    SolverRidge = 1e-9;
    if isfield(Param,'LinearSolverRidge') && Param.LinearSolverRidge > 0
        SolverRidge = Param.LinearSolverRidge;
    end

    % Symmetrize + ridge for safer SPD behavior in PCG.
    A = 0.5 * (II + II') + SolverRidge * speye(nSys);
    x0 = zeros(nSys,1);
    if ~isempty(PCGWarmStart) && ~isempty(PCGWarmSize) && PCGWarmSize == nSys && all(isfinite(PCGWarmStart))
        x0 = PCGWarmStart;
    end

    PCGTol = 1e-3;
    if isfield(Param,'PCGTol') && Param.PCGTol > 0
        PCGTol = Param.PCGTol;
    end
    PCGMaxIter = 80;
    if isfield(Param,'PCGMaxIter') && Param.PCGMaxIter > 0
        PCGMaxIter = floor(Param.PCGMaxIter);
    end
    PCGAcceptTolFactor = 1.10;
    if isfield(Param,'PCGAcceptTolFactor') && Param.PCGAcceptTolFactor >= 1
        PCGAcceptTolFactor = Param.PCGAcceptTolFactor;
    end
    flag = 1;
    relres = inf;
    iter = 0;
    try
        IcholSetup = struct('type','ict', 'droptol',1e-3, 'diagcomp',1e-3);
        Lp = ichol(A, IcholSetup);
        [Delta, flag, relres, iter] = pcg(A, EE, PCGTol, PCGMaxIter, Lp, Lp', x0);
    catch
        [Delta, flag, relres, iter] = pcg(A, EE, PCGTol, PCGMaxIter, [], [], x0);
    end

    IsNearConverged = (flag == 1) && isfinite(relres) && (relres <= PCGAcceptTolFactor * PCGTol);
    if (flag == 0 || IsNearConverged) && all(isfinite(Delta))
        PCGWarmStart = Delta;
        PCGWarmSize = nSys;
    else
        Delta = LocalSolveDirect(II, EE, Param);
        PCGWarmStart = [];
        PCGWarmSize = [];
    end
else
    Delta = LocalSolveDirect(II, EE, Param);
end

nP = size(JP,2);
DeltaP = Delta(1:nP);
DeltaM = Delta(nP+1:end);

Delta = [DeltaP;DeltaM];

if isempty(Delta)
    MeanDelta = 0;
else
    Sum_Delta = Delta'*Delta;
    MeanDelta = full(Sum_Delta/length(Delta));
end
if isempty(DeltaP)
    MeanDeltaPose = 0;
else
    Sum_Delta_Pose = DeltaP'*DeltaP;
    MeanDeltaPose = full(Sum_Delta_Pose/length(DeltaP));
end

end

function Delta = LocalSolveDirect(II, EE, Param)
nSys = size(II,1);
Perm = symamd(II);
Aperm = II(Perm,Perm);
bperm = EE(Perm);

[L,p] = chol(Aperm,'lower');
if p ~= 0
    SolverRidge = 1e-9;
    if isfield(Param,'LinearSolverRidge') && Param.LinearSolverRidge > 0
        SolverRidge = Param.LinearSolverRidge;
    end
    Aperm = Aperm + SolverRidge * speye(nSys);
    [L,p] = chol(Aperm,'lower');
end

if p == 0
    y = L\bperm;
    xperm = L'\y;
else
    xperm = Aperm\bperm;
end

Delta = zeros(nSys,1);
Delta(Perm) = xperm;

if any(~isfinite(Delta))
    SolverRidge = 1e-9;
    if isfield(Param,'LinearSolverRidge') && Param.LinearSolverRidge > 0
        SolverRidge = Param.LinearSolverRidge;
    end
    II = II + SolverRidge * speye(size(II,1));
    Delta = II\EE;
end
if any(~isfinite(Delta))
    Delta(~isfinite(Delta)) = 0;
end
end
