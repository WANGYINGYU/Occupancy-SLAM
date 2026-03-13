function [Map,Param] = FuncInit3DOccupancyMap(Pose,Scan,Param)

UseTruncated = isfield(Param,'UseTruncatedRegionOptimization') && Param.UseTruncatedRegionOptimization;
UseBlockHash = isfield(Param,'UseBlockHashMapInTruncatedMode') && Param.UseBlockHashMapInTruncatedMode;
if UseTruncated && UseBlockHash
    [Map,Param] = FuncInit3DBlockHashMapNorm(Pose,Scan,Param);
    return;
end

NumPose = size(Pose,1);
Scale = Param.Scale;

minXYZ = [inf; inf; inf];
maxXYZ = [-inf; -inf; -inf];
for i = 1:NumPose
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = double(Scan{i}.xyz');
    Si = Ri*xyz+Posei(1:3)';
    minXYZ = min(minXYZ, min(Si,[],2));
    maxXYZ = max(maxXYZ, max(Si,[],2));
end

% find the origin and size of map
min_x = minXYZ(1);
min_y = minXYZ(2);
min_z = minXYZ(3);

if min_x <=1e-6
    int_minx = floor(min_x);
else
    int_minx = fix(min_x);
end

if min_y <=1e-6
    int_miny = floor(min_y);
else
    int_miny = fix(min_y);
end

if min_z <=1e-6
    int_minz = floor(min_z);
else
    int_minz = fix(min_z);
end

Origin = [int_minx - fix(Param.ExtraOrigin(1)/Scale);int_miny- fix(Param.ExtraOrigin(2)/Scale);int_minz- fix(Param.ExtraOrigin(3)/Scale)] ;

% calculate the map size
max_x = round((maxXYZ(1) - Origin(1)) / Scale + 1);
max_y = round((maxXYZ(2) - Origin(2)) / Scale + 1);
max_z = round((maxXYZ(3) - Origin(3)) / Scale + 1);

Param.Origin = Origin;
Size_i = max_y+ fix(Param.ExtraBoundary(1)/Scale);
Size_j = max_x+ fix(Param.ExtraBoundary(2)/Scale);
Size_h = max_z+ fix(Param.ExtraBoundary(3)/Scale);

Param.Size_i = Size_i;
Param.Size_j = Size_j;
Param.Size_h = Size_h;

numVoxels = Size_i * Size_j * Size_h;
GridLin = zeros(numVoxels,1);
NLin = zeros(numVoxels,1);

for i=1:NumPose

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    xyz = Scan{i}.xyz';
    Oddi = Scan{i}.Odd;

    Si = Ri*xyz+Posei(1:3)';

    XYZ3 = (Si-Origin) / Scale + 1;

    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));

    Valid = x >= 1 & x <= Size_j & y >= 1 & y <= Size_i & z >= 1 & z <= Size_h;
    if ~any(Valid)
        continue;
    end

    x = x(Valid);
    y = y(Valid);
    z = z(Valid);
    Oddi = Oddi(Valid)';

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;

    TemN = accumarray(ind',1,[numVoxels,1]);
    IdMore = TemN > 1;
    ValMore = TemN(IdMore);
    TemN(IdMore) = 1;
    NLin = NLin + TemN;

    TemGrid = accumarray(ind',Oddi,[numVoxels,1]);

    TemGrid(IdMore) = TemGrid(IdMore)./ValMore;

    GridLin = GridLin + TemGrid;


end

N = reshape(NLin, [Size_i, Size_j, Size_h]);
Grid = reshape(GridLin, [Size_i, Size_j, Size_h]);

Map.Grid = Grid;
Map.N = N;
Map.Scale = Scale;
Map.Origin = Origin;
Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Size_h = Size_h;

Dgrid = griddedInterpolant(Grid);
Ngrid = griddedInterpolant(N);

Map.DgridG = Dgrid;
Map.NgridG = Ngrid;

[Gdu,Gdv,Gdz] = gradient(Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Gdzgrid = griddedInterpolant(Gdz);

Map.DgridGu = Gdugrid;
Map.DgridGv = Gdvgrid;
Map.DgridGz = Gdzgrid;



end
