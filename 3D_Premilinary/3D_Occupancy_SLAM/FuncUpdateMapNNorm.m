function Map = FuncUpdateMapNNorm(Map,Pose,Scan)

NumPose = size(Pose,1);

Scale = Map.Scale;
Origin = Map.Origin;

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

NumVoxel = Size_i * Size_j * Size_h;
cell_UniInd = cell(NumPose,1);

% Project each pose independently, keep one hit per pose/voxel.
parfor i = 1:NumPose
    cell_UniInd{i} = zeros(0,1,'uint32');

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    xyz = Scan{i}.xyz';

    Si = Ri*xyz+Posei(1:3)';

    XYZ3 = (Si-Origin) / Scale + 1;
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));

    Valid = x >= 1 & x <= Size_j & ...
            y >= 1 & y <= Size_i & ...
            z >= 1 & z <= Size_h;
    if ~any(Valid)
        continue;
    end

    x = x(Valid);
    y = y(Valid);
    z = z(Valid);

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;
    cell_UniInd{i} = uint32(unique(ind(:)));

end

N = zeros(NumVoxel,1);
ChunkSize = 32;
for s = 1:ChunkSize:NumPose
    e = min(s + ChunkSize - 1, NumPose);
    AllIndChunk = vertcat(cell_UniInd{s:e});
    if isempty(AllIndChunk)
        continue;
    end
    [UniChunk, ~, Ic] = unique(AllIndChunk);
    HitCnt = accumarray(double(Ic), 1);
    N(double(UniChunk)) = N(double(UniChunk)) + HitCnt;
end
N = reshape(N, [Size_i, Size_j, Size_h]);

Map.N = N;

end
