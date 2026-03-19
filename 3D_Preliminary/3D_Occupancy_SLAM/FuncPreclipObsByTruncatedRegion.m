function [ObsOut, Stats] = FuncPreclipObsByTruncatedRegion(ObsIn, Pose, Map, Param)
% One-time observation preclip by truncated region (+tolerance layers).
% Keep only observations whose 8-neighbor map cells intersect clipped region.

NumPose = length(ObsIn);
ObsOut = ObsIn;

Stats = struct('NumInput',0,'NumOutput',0,'CompressionRatio',0,'ReductionRatio',0);
if NumPose == 0
    return;
end

ExtraLayers = 2;
if isfield(Param,'TruncatedObsClipExtraLayers') && Param.TruncatedObsClipExtraLayers >= 0
    ExtraLayers = floor(Param.TruncatedObsClipExtraLayers);
end

ParamClip = Param;
if ~isfield(ParamClip,'TruncatedRegionLayers') || ParamClip.TruncatedRegionLayers < 0
    ParamClip.TruncatedRegionLayers = 0;
end
ParamClip.TruncatedRegionLayers = ParamClip.TruncatedRegionLayers + ExtraLayers;

KeepVarId = FuncFindCellOptimized(Map, ParamClip);
HitSeedVarId = LocalCollectHitSeedVarId(ObsIn, Pose, Map);
if ~isempty(HitSeedVarId)
    KeepVarId = unique([double(KeepVarId(:)); double(HitSeedVarId(:))]);
end
if isempty(KeepVarId)
    for i = 1:NumPose
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
        ObsOut{i}.HitXYZ = zeros(0,3);
    end
    return;
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
Origin = Map.Origin;
Scale = Map.Scale;
WorldToMapR = eye(3);
WorldToMapT = zeros(3,1);
if isfield(Map,'WorldToMapR') && isequal(size(Map.WorldToMapR),[3,3])
    WorldToMapR = double(Map.WorldToMapR);
end
if isfield(Map,'WorldToMapT') && numel(Map.WorldToMapT) == 3
    WorldToMapT = double(Map.WorldToMapT(:));
end
TotalSize = Size_i * Size_j * Size_h;

KeepMask = false(TotalSize,1);
KeepMask(KeepVarId) = true;

NumInput = 0;
NumOutput = 0;
NumInputEach = zeros(NumPose,1);
NumOutputEach = zeros(NumPose,1);

parfor i = 1:NumPose
    NumInputEach(i) = 0;
    NumOutputEach(i) = 0;

    if ~isfield(ObsIn{i},'xyz') || isempty(ObsIn{i}.xyz)
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
        ObsOut{i}.HitXYZ = zeros(0,3);
        continue;
    end

    xyz = double(ObsIn{i}.xyz);
    Odd = ObsIn{i}.Odd(:);
    if size(xyz,2) > 3
        xyz = xyz(:,1:3);
    end
    if size(xyz,2) ~= 3
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
        ObsOut{i}.HitXYZ = zeros(0,3);
        continue;
    end

    nPts = size(xyz,1);
    if length(Odd) ~= nPts
        nUse = min(length(Odd), nPts);
        xyz = xyz(1:nUse,:);
        Odd = Odd(1:nUse);
        nPts = nUse;
    end
    NumInputEach(i) = nPts;
    if nPts == 0
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
        ObsOut{i}.HitXYZ = zeros(0,3);
        continue;
    end

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');

    Si = Ri * xyz' + Posei(1:3)';
    Si = FuncWorldToMapFrame(Si, WorldToMapR, WorldToMapT);
    Pi = (Si - Origin) / Scale + 1;

    u = Pi(1,:);
    v = Pi(2,:);
    h = Pi(3,:);

    % Interior points use 8-neighbor test; boundary points fall back to
    % nearest-cell keep test to avoid false drops at clipped-map borders.
    Tol = 1e-6;
    ValidNear = all(isfinite(Pi),1) & ...
                u >= 1-Tol & u <= Size_j+Tol & ...
                v >= 1-Tol & v <= Size_i+Tol & ...
                h >= 1-Tol & h <= Size_h+Tol;
    ValidInterp = ValidNear & ...
                  u < Size_j-Tol & ...
                  v < Size_i-Tol & ...
                  h < Size_h-Tol;

    KeepPoint = false(1,nPts);
    if any(ValidInterp)
        u1 = fix(u(ValidInterp));
        v1 = fix(v(ValidInterp));
        h1 = fix(h(ValidInterp));

        m1 = Size_j*(v1-1)+u1 + (h1-1)*Size_i*Size_j;
        m2 = Size_j*v1+u1 + (h1-1)*Size_i*Size_j;
        m3 = Size_j*(v1-1)+u1+1 + (h1-1)*Size_i*Size_j;
        m4 = Size_j*v1+u1+1 + (h1-1)*Size_i*Size_j;
        m5 = Size_j*(v1-1)+u1 + h1*Size_i*Size_j;
        m6 = Size_j*v1+u1 + h1*Size_i*Size_j;
        m7 = Size_j*(v1-1)+u1+1 + h1*Size_i*Size_j;
        m8 = Size_j*v1+u1+1 + h1*Size_i*Size_j;

        KeepInterp = KeepMask(m1) | KeepMask(m2) | KeepMask(m3) | KeepMask(m4) | ...
                     KeepMask(m5) | KeepMask(m6) | KeepMask(m7) | KeepMask(m8);
        ValidId = find(ValidInterp);
        KeepPoint(ValidId(KeepInterp)) = true;
    end

    ValidBoundary = ValidNear & ~ValidInterp;
    if any(ValidBoundary)
        ub = round(u(ValidBoundary));
        vb = round(v(ValidBoundary));
        hb = round(h(ValidBoundary));

        ub = min(max(ub,1),Size_j);
        vb = min(max(vb,1),Size_i);
        hb = min(max(hb,1),Size_h);

        mb = Size_j*(vb-1)+ub + (hb-1)*Size_i*Size_j;
        KeepBoundary = KeepMask(mb);
        BoundaryId = find(ValidBoundary);
        KeepPoint(BoundaryId(KeepBoundary)) = true;
    end

    KeepPointCol = KeepPoint(:);
    ObsOut{i}.xyz = xyz(KeepPointCol,:);
    ObsOut{i}.Odd = Odd(KeepPointCol);
    HitMask = KeepPointCol & (Odd > 0);
    ObsOut{i}.HitXYZ = xyz(HitMask,:);
    NumOutputEach(i) = nnz(KeepPointCol);
end

NumInput = sum(NumInputEach);
NumOutput = sum(NumOutputEach);

Stats.NumInput = NumInput;
Stats.NumOutput = NumOutput;
Stats.CompressionRatio = NumOutput / max(NumInput,1);
Stats.ReductionRatio = 1 - Stats.CompressionRatio;

end

function HitSeedVarId = LocalCollectHitSeedVarId(ObsIn, Pose, Map)
NumPose = length(ObsIn);
if NumPose == 0 || isempty(Pose)
    HitSeedVarId = [];
    return;
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
Origin = Map.Origin;
Scale = Map.Scale;
WorldToMapR = eye(3);
WorldToMapT = zeros(3,1);
if isfield(Map,'WorldToMapR') && isequal(size(Map.WorldToMapR),[3,3])
    WorldToMapR = double(Map.WorldToMapR);
end
if isfield(Map,'WorldToMapT') && numel(Map.WorldToMapT) == 3
    WorldToMapT = double(Map.WorldToMapT(:));
end

IdCell = cell(NumPose,1);
for i = 1:NumPose
    IdCell{i} = zeros(0,1);
    if i > size(Pose,1) || ~isstruct(ObsIn{i})
        continue;
    end

    if isfield(ObsIn{i},'HitXYZ') && ~isempty(ObsIn{i}.HitXYZ)
        HitLocal = double(ObsIn{i}.HitXYZ);
    elseif isfield(ObsIn{i},'xyz') && ~isempty(ObsIn{i}.xyz) && ...
           isfield(ObsIn{i},'Odd') && ~isempty(ObsIn{i}.Odd)
        xyz = double(ObsIn{i}.xyz);
        Odd = ObsIn{i}.Odd(:);
        nUse = min(size(xyz,1), length(Odd));
        xyz = xyz(1:nUse,:);
        Odd = Odd(1:nUse);
        HitLocal = xyz(Odd > 0,:);
    else
        continue;
    end

    if isempty(HitLocal)
        continue;
    end
    if size(HitLocal,2) > 3
        HitLocal = HitLocal(:,1:3);
    end
    if size(HitLocal,2) ~= 3
        continue;
    end

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');

    Si = (Ri * HitLocal' + Posei(1:3)')';
    Si = FuncWorldToMapFrame(Si, WorldToMapR, WorldToMapT);
    Pi = (Si - Origin') / Scale + 1;

    x = round(Pi(:,1));
    y = round(Pi(:,2));
    z = round(Pi(:,3));

    Valid = isfinite(x) & isfinite(y) & isfinite(z) & ...
            x >= 1 & x <= Size_j & ...
            y >= 1 & y <= Size_i & ...
            z >= 1 & z <= Size_h;
    if ~any(Valid)
        continue;
    end

    x = x(Valid);
    y = y(Valid);
    z = z(Valid);
    IdCell{i} = Size_j*(y-1) + x + (z-1)*Size_i*Size_j;
end

HitSeedVarId = unique(vertcat(IdCell{:}));
end
