function Map = FuncInit3DOccupancyMap(Pose,Scan,Param)

NumPose = size(Pose,1);
Scale = Param.Scale;


% Origin = Param.Origin;
% Scale = Param.Scale;
% Size_i = Param.Size_i;
% Size_j = Param.Size_j;
% Size_h = Param.Size_h;
% 
% Grid = zeros(Size_i, Size_j, Size_h);
% N = zeros(Size_i, Size_j, Size_h);


% project 3D local observations into global coordinate
PointsGlobal = [];
OddAll = [];
for i = 1:NumPose

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    xyz = Scan{i}.xyz';
    Oddi = Scan{i}.Odd;

    Si = Ri*xyz+Posei(1:3)';
    PointsGlobal = [PointsGlobal,Si];
    OddAll = [OddAll;Oddi];
end

% find the origin and size of map
min_x = min(PointsGlobal(1,:));
min_y = min(PointsGlobal(2,:));
min_z = min(PointsGlobal(3,:));

if min_x <=0
    int_minx = floor(min_x)-1;
else
    int_minx = fix(min_x)+1;
end

if min_y <=0
    int_miny = floor(min_y)-1;
else
    int_miny = fix(min_y)+1;
end

if min_z <=0
    int_minz = floor(min_z)-1;
else
    int_minz = fix(min_z)+1;
end

Origin = [int_minx;int_miny;int_minz];

XYZ3 = (PointsGlobal-Origin) / Scale + 1;

    % XYZ3 = (Si-Origin) / Scale + 1;
x = round(XYZ3(1,:));
y = round(XYZ3(2,:));
z = round(XYZ3(3,:));

% calculate the map size
max_x = max(x);
max_y = max(y);
max_z = max(z);

Param.Origin = Origin;
Size_i = max_y+1/Scale;
Size_j = max_x+1/Scale;
Size_h = max_z+1/Scale;

Param.Size_i = Size_i;
Param.Size_j = Size_j;
Param.Size_h = Size_h;



Grid = zeros(Size_i, Size_j, Size_h);
N = zeros(Size_i, Size_j, Size_h);


% if sum((x<=0).*(y<=0).*(z<=0))~=0
%     pfintf("Error Size \n\n");
% end

ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;

% ind = sub2ind(size(Grid), y, x, z);
% TemGrid = zeros(Size_i, Size_j, Size_v);
% TemGrid(ind) = Oddi;
TemGrid = accumarray(ind',OddAll,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);
TemGrid_reshaped = reshape(TemGrid, size(Grid));

Grid = Grid + TemGrid_reshaped;

TemN = accumarray(ind',1,[size(N,1)*size(N,2)*size(N,3),1]);
TemN_reshaped = reshape(TemN, size(N));

N = N + TemN_reshaped;

% end    

Map.Grid = Grid;
Map.N = N;
Map.Scale = Scale;
Map.Origin = Origin;
Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Size_h = Size_h;

% NormalizeMap = Grid./N;
% NormalizeMap(isnan(NormalizeMap)) = 0;
% 
% Map.NormalizeMap = NormalizeMap;


Dgrid = griddedInterpolant(Grid);
Ngrid = griddedInterpolant(N);

% NormalizeDgrid = griddedInterpolant(NormalizeMap);


Map.DgridG = Dgrid;
Map.NgridG = Ngrid;
% Map.NormalizeDgrid = NormalizeDgrid;

[Gdu,Gdv,Gdz] = gradient(Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Gdzgrid = griddedInterpolant(Gdz);

Map.DgridGu = Gdugrid;
Map.DgridGv = Gdvgrid;
Map.DgridGz = Gdzgrid;

% [NormalizeGdu,NormalizeGdv,NormalizeGdz] = gradient(NormalizeMap);
% 
% NormalizeGdugrid = griddedInterpolant(NormalizeGdu);
% NormalizeGdvgrid = griddedInterpolant(NormalizeGdv);
% NormalizeGdzgrid = griddedInterpolant(NormalizeGdz);
% 
% Map.NormalizeGdugrid = NormalizeGdugrid;
% Map.NormalizeGdvgrid = NormalizeGdvgrid;
% Map.NormalizeGdzgrid = NormalizeGdzgrid;

end
