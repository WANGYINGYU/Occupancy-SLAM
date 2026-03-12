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
if isempty(KeepVarId)
    for i = 1:NumPose
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
    end
    return;
end

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;
Origin = Map.Origin;
Scale = Map.Scale;
TotalSize = Size_i * Size_j * Size_h;

KeepMask = false(TotalSize,1);
KeepMask(KeepVarId) = true;

NumInput = 0;
NumOutput = 0;

for i = 1:NumPose
    if ~isfield(ObsIn{i},'xyz') || isempty(ObsIn{i}.xyz)
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
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
        continue;
    end

    nPts = size(xyz,1);
    if length(Odd) ~= nPts
        nUse = min(length(Odd), nPts);
        xyz = xyz(1:nUse,:);
        Odd = Odd(1:nUse);
        nPts = nUse;
    end
    NumInput = NumInput + nPts;
    if nPts == 0
        ObsOut{i}.xyz = zeros(0,3);
        ObsOut{i}.Odd = zeros(0,1);
        continue;
    end

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');

    Si = Ri * xyz' + Posei(1:3)';
    Pi = (Si - Origin) / Scale + 1;

    u = Pi(1,:);
    v = Pi(2,:);
    h = Pi(3,:);

    % Need valid 8-neighbor interpolation support.
    Valid = all(isfinite(Pi),1) & ...
            u >= 1 & u < Size_j & ...
            v >= 1 & v < Size_i & ...
            h >= 1 & h < Size_h;

    KeepPoint = false(1,nPts);
    if any(Valid)
        u1 = fix(u(Valid));
        v1 = fix(v(Valid));
        h1 = fix(h(Valid));

        m1 = Size_j*(v1-1)+u1 + (h1-1)*Size_i*Size_j;
        m2 = Size_j*v1+u1 + (h1-1)*Size_i*Size_j;
        m3 = Size_j*(v1-1)+u1+1 + (h1-1)*Size_i*Size_j;
        m4 = Size_j*v1+u1+1 + (h1-1)*Size_i*Size_j;
        m5 = Size_j*(v1-1)+u1 + h1*Size_i*Size_j;
        m6 = Size_j*v1+u1 + h1*Size_i*Size_j;
        m7 = Size_j*(v1-1)+u1+1 + h1*Size_i*Size_j;
        m8 = Size_j*v1+u1+1 + h1*Size_i*Size_j;

        KeepValid = KeepMask(m1) | KeepMask(m2) | KeepMask(m3) | KeepMask(m4) | ...
                    KeepMask(m5) | KeepMask(m6) | KeepMask(m7) | KeepMask(m8);
        ValidId = find(Valid);
        KeepPoint(ValidId(KeepValid)) = true;
    end

    KeepPointCol = KeepPoint(:);
    ObsOut{i}.xyz = xyz(KeepPointCol,:);
    ObsOut{i}.Odd = Odd(KeepPointCol);
    NumOutput = NumOutput + nnz(KeepPointCol);
end

Stats.NumInput = NumInput;
Stats.NumOutput = NumOutput;
Stats.CompressionRatio = NumOutput / max(NumInput,1);
Stats.ReductionRatio = 1 - Stats.CompressionRatio;

end
