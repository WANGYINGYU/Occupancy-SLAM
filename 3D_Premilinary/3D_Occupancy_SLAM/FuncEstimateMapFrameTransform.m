function [WorldToMapR, WorldToMapT, Stats] = FuncEstimateMapFrameTransform(Pose, Scan, Param)
% Estimate a fixed world->map rotation for tighter axis-aligned map bounds.
% Current mode: yaw PCA on projected world points.

WorldToMapR = eye(3);
WorldToMapT = zeros(3,1);
Stats = struct('Enabled',false,'YawRad',0,'NumWorldPoints',0);

if nargin < 3 || isempty(Param)
    Param = struct();
end
if ~isfield(Param,'UseMapFrameRotation') || ~Param.UseMapFrameRotation
    return;
end

Mode = 'yaw_pca';
if isfield(Param,'MapFrameRotationMode') && ~isempty(Param.MapFrameRotationMode)
    Mode = lower(char(Param.MapFrameRotationMode));
end
if ~strcmp(Mode,'yaw_pca')
    warning('Unknown MapFrameRotationMode=%s. Fallback to yaw_pca.', Mode);
end

NumPose = size(Pose,1);
if NumPose == 0 || isempty(Scan)
    return;
end

SamplePerScan = 300;
if isfield(Param,'MapFrameSamplePerScan') && Param.MapFrameSamplePerScan > 0
    SamplePerScan = floor(Param.MapFrameSamplePerScan);
end

WorldXYCell = cell(NumPose,1);
for i = 1:NumPose
    WorldXYCell{i} = zeros(0,2);
    if i > numel(Scan)
        continue;
    end

    PLocal = LocalExtractPointsXYZ(Scan{i});
    if isempty(PLocal)
        continue;
    end

    nPts = size(PLocal,1);
    if nPts > SamplePerScan
        Id = round(linspace(1,nPts,SamplePerScan));
        PLocal = PLocal(Id,:);
    end

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');

    PWorld = (Ri * PLocal' + Posei(1:3)')';
    Valid = all(isfinite(PWorld),2);
    PWorld = PWorld(Valid,:);
    if isempty(PWorld)
        continue;
    end
    WorldXYCell{i} = PWorld(:,1:2);
end

WorldXY = vertcat(WorldXYCell{:});
if size(WorldXY,1) < 20
    % Fallback: use trajectory xy distribution.
    if size(Pose,1) >= 3
        PoseXY = Pose(:,1:2);
        ValidP = all(isfinite(PoseXY),2);
        WorldXY = PoseXY(ValidP,:);
    end
end

if size(WorldXY,1) < 3
    return;
end

Mu = mean(WorldXY,1);
X = WorldXY - Mu;
C = (X' * X) / max(size(X,1)-1,1);
[V,D] = eig(C);
[~,IdMax] = max(diag(D));
Axis1 = V(:,IdMax);
Yaw = atan2(Axis1(2), Axis1(1));

cy = cos(-Yaw);
sy = sin(-Yaw);
WorldToMapR = [cy, -sy, 0;
               sy,  cy, 0;
                0,   0, 1];
WorldToMapT = zeros(3,1);

Stats.Enabled = true;
Stats.YawRad = Yaw;
Stats.NumWorldPoints = size(WorldXY,1);
end

function P = LocalExtractPointsXYZ(ScanEntry)
% Return Nx3 local xyz points from raw scan or processed observation.

P = zeros(0,3);
if isempty(ScanEntry)
    return;
end

if isstruct(ScanEntry)
    if isfield(ScanEntry,'HitXYZ') && ~isempty(ScanEntry.HitXYZ)
        P = double(ScanEntry.HitXYZ);
    elseif isfield(ScanEntry,'xyz') && ~isempty(ScanEntry.xyz)
        P = double(ScanEntry.xyz);
        if isfield(ScanEntry,'Odd') && ~isempty(ScanEntry.Odd)
            Odd = ScanEntry.Odd(:);
            nUse = min(size(P,1), length(Odd));
            P = P(1:nUse,:);
            Odd = Odd(1:nUse);
            HitId = Odd > 0;
            if any(HitId)
                P = P(HitId,:);
            end
        end
    end
else
    Raw = double(ScanEntry);
    [nRow, nCol] = size(Raw);
    if nRow >= 3 && nCol > nRow
        P = Raw(1:3,:)';
    elseif nCol >= 3
        P = Raw(:,1:3);
    end
end

if isempty(P)
    return;
end
if size(P,2) > 3
    P = P(:,1:3);
end
if size(P,2) ~= 3
    P = zeros(0,3);
    return;
end
Valid = all(isfinite(P),2);
P = P(Valid,:);
end
