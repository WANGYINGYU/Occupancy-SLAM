function [DownPoints, Stats] = FuncVoxelGridDownsample(Points, VoxelSize, Method, Verbose)
% Downsample point cloud by voxel grid.
% Input supports D x N (points in columns) or N x D (points in rows).
% VoxelSize is in meters. Method: 'centroid' (default) or 'first'.

if nargin < 3 || isempty(Method)
    Method = 'centroid';
end
if nargin < 4 || isempty(Verbose)
    Verbose = false;
end

if isempty(Points)
    DownPoints = Points;
    Stats = struct('NumInputRaw',0,'NumInputValid',0,'NumOutput',0, ...
                   'CompressionRatio',0,'ReductionRatio',0);
    return;
end

if ~isscalar(VoxelSize) || ~isfinite(VoxelSize) || VoxelSize <= 0
    error('VoxelSize must be a positive scalar.');
end

[nRow, nCol] = size(Points);

if nRow >= 3 && nRow <= 8 && nCol > nRow
    IsColumnFormat = true;
    P = double(Points');
elseif nCol >= 3
    IsColumnFormat = false;
    P = double(Points);
else
    error('Points must be D x N or N x D with at least 3 spatial dimensions.');
end

XYZ = P(:,1:3);
NumInputRaw = size(P,1);
ValidId = all(isfinite(XYZ), 2);
P = P(ValidId,:);
XYZ = XYZ(ValidId,:);
NumInputValid = size(P,1);

if isempty(P)
    if IsColumnFormat
        DownPoints = zeros(nRow, 0);
    else
        DownPoints = zeros(0, nCol);
    end
    Stats = struct('NumInputRaw',NumInputRaw,'NumInputValid',0,'NumOutput',0, ...
                   'CompressionRatio',0,'ReductionRatio',0);
    return;
end

VoxelIdx = floor(XYZ ./ VoxelSize);

switch lower(Method)
    case 'first'
        [~, FirstId] = unique(VoxelIdx, 'rows', 'stable');
        DownP = P(FirstId,:);
    case 'centroid'
        [~, ~, GroupId] = unique(VoxelIdx, 'rows', 'stable');
        NumVoxel = max(GroupId);
        DownP = zeros(NumVoxel, size(P,2));
        for d = 1:size(P,2)
            DownP(:,d) = accumarray(GroupId, P(:,d), [NumVoxel,1], @mean);
        end
    otherwise
        error('Unknown Method. Use ''centroid'' or ''first''.');
end

if IsColumnFormat
    DownPoints = DownP';
else
    DownPoints = DownP;
end

NumOutput = size(DownP,1);
CompressionRatio = NumOutput / max(NumInputValid,1);
ReductionRatio = 1 - CompressionRatio;
Stats = struct('NumInputRaw',NumInputRaw,'NumInputValid',NumInputValid, ...
               'NumOutput',NumOutput,'CompressionRatio',CompressionRatio, ...
               'ReductionRatio',ReductionRatio);

if Verbose
    fprintf('[VoxelDS] in(raw/valid)=%d/%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
            NumInputRaw, NumInputValid, NumOutput, CompressionRatio, 100*ReductionRatio);
end

end
