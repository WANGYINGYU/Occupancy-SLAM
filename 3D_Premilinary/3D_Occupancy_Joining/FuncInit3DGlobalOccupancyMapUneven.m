function [Map,LocalObs,Param] = FuncInit3DGlobalOccupancyMapUneven(Pose,Submap,Param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Build Global Occupancy Map from Local Submaps
% Code Witten by Yingyu Wang
% 30/05/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


NumPose = size(Pose,1);
GlobalScaleX = Param.GlobalScaleX;
GlobalScaleY = Param.GlobalScaleY;
GlobalScaleZ = Param.GlobalScaleZ;

LocalScaleX = Param.LocalScaleX;
LocalScaleY = Param.LocalScaleY;
LocalScaleZ = Param.LocalScaleZ;

LocalObs = cell(1,NumPose);

PointsGlobal = cell(1,NumPose);

for i=1:NumPose
    % Generate Observations from Local Submaps
    Submapi = Submap{i};
    SubMapOrigin = Submapi.Origin;
    % Id = find(Submapi.Grid~=0);
    Id = find(Submapi.OriginalGrid~=0);
    
    [I,J,H] = ind2sub(size(Submapi.Grid),Id);
    
    % Oddi = Submapi.DgridG(I+0,J+0,H+0);
    Oddi = Submapi.OriginalDgrid(I+0,J+0,H+0);

    % SparseId = FuncCheckGridSparse(Id,size(Submapi.Grid),1,Submapi.N);
    
    % Ni = Submapi.N(Id);
    % Ni = Submapi.NgridG(I+0,J+0,H+0);
    Ni = Submapi.OriginalNgrid(I+0,J+0,H+0);
    MapPoints = [J,I,H]-1;
    XYZi = [MapPoints(:,1)*LocalScaleX,MapPoints(:,2)*LocalScaleY,MapPoints(:,3)*LocalScaleZ] + SubMapOrigin';

    % XYZi(SparseId,:) = [];
    % Oddi(SparseId) = [];
    % Ni(SparseId) = [];

    LocalObs{i}.xyz = XYZi;
    LocalObs{i}.Odd = Oddi;
    LocalObs{i}.N = Ni;
    
    % Project to Global Map
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));  
    Ri = FuncR(RZ',RY',RX');
    Si = Ri*XYZi'+Posei(1:3)';
    PointsGlobal{i} = Si;
end

AllPoints = horzcat(PointsGlobal{:});

% find the origin and size of map
min_x = min(AllPoints(1,:));
min_y = min(AllPoints(2,:));
min_z = min(AllPoints(3,:));

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

GlobalOrigin = [int_minx-1;int_miny-1;int_minz-2];

Param.GlobalOrigin = GlobalOrigin;

XYZ = (AllPoints-GlobalOrigin);

XYZ3 = [XYZ(1,:)/GlobalScaleX;XYZ(2,:)/GlobalScaleY;XYZ(3,:)/GlobalScaleZ] + 1;

x = round(XYZ3(1,:));
y = round(XYZ3(2,:));
z = round(XYZ3(3,:));

% calculate the map size
max_x = max(x);
max_y = max(y);
max_z = max(z);

Size_i = max_y+4/GlobalScaleY;
Size_j = max_x+4/GlobalScaleX;
Size_h = max_z+8/GlobalScaleZ;

Param.GlobalSize_i = Size_i;
Param.GlobalSize_j = Size_j;
Param.GlobalSize_h = Size_h;

Grid = zeros(Size_i, Size_j, Size_h);
N = zeros(Size_i, Size_j, Size_h);


for i=1:NumPose
    Si = PointsGlobal{i};
    Oddi = LocalObs{i}.Odd;
    Ni = LocalObs{i}.N;
    XYZ = (Si-GlobalOrigin);
    XYZ3 = [XYZ(1,:)/GlobalScaleX;XYZ(2,:)/GlobalScaleY;XYZ(3,:)/GlobalScaleZ] + 1;

    % Assign Value by trilinear interpolation
    % u = XYZ3(1,:);
    % v = XYZ3(2,:);
    % h = XYZ3(3,:);
    % 
    % u1 = fix(u);
    % v1 = fix(v);
    % h1 = fix(h);
    % 
    % % Calculate the Weights for 8 points (for 3D case)
    % w1 = (v1+1-v).*(u1+1-u).*(h1+1-h);
    % w2 = (v-v1).*(u1+1-u).*(h1+1-h);
    % w3 = (v1+1-v).*(u-u1).*(h1+1-h);
    % w4 = (v-v1).*(u-u1).*(h1+1-h);
    % 
    % w5 = (v1+1-v).*(u1+1-u).*(h-h1);
    % w6 = (v-v1).*(u1+1-u).*(h-h1);
    % w7 = (v1+1-v).*(u-u1).*(h-h1);
    % w8 = (v-v1).*(u-u1).*(h-h1);
    % 
    % ID1 = sub2ind(size(N),v1,u1,h1);
    % ID2 = sub2ind(size(N),v1+1,u1,h1);
    % ID3 = sub2ind(size(N),v1,u1+1,h1);
    % ID4 = sub2ind(size(N),v1+1,u1+1,h1);
    % ID5 = sub2ind(size(N),v1,u1,h1+1);
    % ID6 = sub2ind(size(N),v1+1,u1,h1+1);
    % ID7 = sub2ind(size(N),v1,u1+1,h1+1);
    % ID8 = sub2ind(size(N),v1+1,u1+1,h1+1);
    % 
    % Ind = [ID1';ID2';ID3';ID4';ID5';ID6';ID7';ID8'];
    % 
    % WeightedNi = [w1';w2';w3';w4';w5';w6';w7';w8'] .* repmat(Ni,8,1);
    % 
    % TemN = accumarray(Ind,WeightedNi,[size(N,1)*size(N,2)*size(N,3),1]);
    % TemNReshaped = reshape(TemN, size(N));
    % N = N + TemNReshaped; 
    % 
    % WeightedOddi = [w1';w2';w3';w4';w5';w6';w7';w8'] .* repmat(Oddi,8,1);
    % TemGrid = accumarray(Ind,WeightedOddi,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);
    % TemGridReshaped = reshape(TemGrid, size(Grid));
    % Grid = Grid + TemGridReshaped;


    % Assign Value to the Nearest Cell
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));
    Ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;
    TemN = accumarray(Ind',Ni,[size(N,1)*size(N,2)*size(N,3),1]);
    TemN_reshaped = reshape(TemN, size(N));
    N = N + TemN_reshaped; 
    TemGrid = accumarray(Ind',Oddi,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);
    TemGrid_reshaped = reshape(TemGrid, size(Grid));
    Grid = Grid + TemGrid_reshaped;
end

Check = Grid./N;
Grid(abs(Check)>=1000) = 0;
N(abs(Check)>=1000) = 0;

% Map.Scale = GlobalScale;
Map.Origin = GlobalOrigin;
Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Size_h = Size_h;

Map.Grid = Grid;
Map.N = N;

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
