function [Map,Param] = FuncInit3DOccupancyMap(Pose,Scan,Param)

NumPose = size(Pose,1);
Scale = Param.Scale;

PointsGlobal = [];
parfor i = 1:NumPose
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = double(Scan{i}.xyz');
    Si = Ri*xyz+Posei(1:3)';
    PointsGlobal = [PointsGlobal,Si];
end

% find the origin and size of map
min_x = min(PointsGlobal(1,:));
min_y = min(PointsGlobal(2,:));
min_z = min(PointsGlobal(3,:));

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

Origin = [int_minx - fix(1/Scale);int_miny- fix(1/Scale);int_minz- fix(4/Scale)] ;

XYZ3 = (PointsGlobal-Origin) / Scale + 1;

x = round(XYZ3(1,:));
y = round(XYZ3(2,:));
z = round(XYZ3(3,:));

% calculate the map size
max_x = max(x);
max_y = max(y);
max_z = max(z);

Param.Origin = Origin;
Size_i = max_y+ fix(2/Scale);
Size_j = max_x+ fix(2/Scale);
Size_h = max_z+ fix(8/Scale);

Param.Size_i = Size_i;
Param.Size_j = Size_j;
Param.Size_h = Size_h;

Grid = zeros(Size_i, Size_j, Size_h);
N = zeros(Size_i, Size_j, Size_h);

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

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;

    TemN = accumarray(ind',1,[size(N,1)*size(N,2)*size(N,3),1]);
    IdMore = find(TemN>1);
    ValMore = TemN(IdMore);
    TemN(IdMore) = 1;
    TemN_reshaped = reshape(TemN, size(N));
    N = N + TemN_reshaped;

    TemGrid = accumarray(ind',Oddi,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);

    TemGrid(IdMore) = TemGrid(IdMore)./ValMore;

    TemGrid_reshaped = reshape(TemGrid, size(Grid));    
    Grid = Grid + TemGrid_reshaped;


end    

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
