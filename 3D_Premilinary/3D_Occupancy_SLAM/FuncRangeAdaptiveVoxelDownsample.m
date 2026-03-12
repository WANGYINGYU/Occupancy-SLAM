function [DownPoints, Stats] = FuncRangeAdaptiveVoxelDownsample(Points, RangeEdges, VoxelSizes, Method, Verbose)
% Distance-adaptive voxel downsample in local frame.
% Each range bin [RangeEdges(k), RangeEdges(k+1)) uses VoxelSizes(k).

if nargin < 4 || isempty(Method)
    Method = 'centroid';
end
if nargin < 5 || isempty(Verbose)
    Verbose = false;
end

if isempty(Points)
    DownPoints = Points;
    Stats = struct('NumInputRaw',0,'NumInputValid',0,'NumOutput',0, ...
                   'CompressionRatio',0,'ReductionRatio',0);
    return;
end

if ~isvector(RangeEdges) || ~isvector(VoxelSizes)
    error('RangeEdges and VoxelSizes must be vectors.');
end

RangeEdges = double(RangeEdges(:)');
VoxelSizes = double(VoxelSizes(:)');

if numel(RangeEdges) ~= numel(VoxelSizes) + 1
    error('numel(RangeEdges) must equal numel(VoxelSizes)+1.');
end
if any(~isfinite(RangeEdges(1:end-1))) || any(~isfinite(VoxelSizes))
    error('RangeEdges (except final +inf) and VoxelSizes must be finite.');
end
if any(diff(RangeEdges) <= 0)
    error('RangeEdges must be strictly increasing.');
end
if any(VoxelSizes <= 0)
    error('VoxelSizes must be positive.');
end

[nRow, nCol] = size(Points);
if nRow >= 3 && nRow <= 8 && nCol > nRow
    IsColumnFormat = true;
    P = double(Points);
elseif nCol >= 3
    IsColumnFormat = false;
    P = double(Points');
else
    error('Points must be D x N or N x D with at least 3 spatial dimensions.');
end

NumInputRaw = size(P,2);
ValidId = all(isfinite(P(1:3,:)),1);
P = P(:,ValidId);
NumInputValid = size(P,2);

if isempty(P)
    if IsColumnFormat
        DownPoints = zeros(nRow,0);
    else
        DownPoints = zeros(0,nCol);
    end
    Stats = struct('NumInputRaw',NumInputRaw,'NumInputValid',0,'NumOutput',0, ...
                   'CompressionRatio',0,'ReductionRatio',0);
    return;
end

Range = sqrt(sum(P(1:3,:).^2,1));
DownCells = cell(1,numel(VoxelSizes));
Count = 0;

for k = 1:numel(VoxelSizes)
    Low = RangeEdges(k);
    High = RangeEdges(k+1);
    if k < numel(VoxelSizes)
        Id = (Range >= Low) & (Range < High);
    else
        Id = (Range >= Low) & (Range <= High);
    end
    if ~any(Id)
        continue;
    end
    [DownK, ~] = FuncVoxelGridDownsample(P(:,Id), VoxelSizes(k), Method, false);
    Count = Count + 1;
    DownCells{Count} = DownK;
end

if Count == 0
    DownP = zeros(size(P,1),0);
else
    DownP = [DownCells{1:Count}];
end

NumOutput = size(DownP,2);
CompressionRatio = NumOutput / max(NumInputValid,1);
ReductionRatio = 1 - CompressionRatio;
Stats = struct('NumInputRaw',NumInputRaw,'NumInputValid',NumInputValid, ...
               'NumOutput',NumOutput,'CompressionRatio',CompressionRatio, ...
               'ReductionRatio',ReductionRatio);

if Verbose
    fprintf('[VoxelDS-Adapt] in(raw/valid)=%d/%d, out=%d, compression=%.4f, reduction=%.2f%%\n', ...
            NumInputRaw, NumInputValid, NumOutput, CompressionRatio, 100*ReductionRatio);
end

if IsColumnFormat
    DownPoints = DownP;
else
    DownPoints = DownP';
end

end
