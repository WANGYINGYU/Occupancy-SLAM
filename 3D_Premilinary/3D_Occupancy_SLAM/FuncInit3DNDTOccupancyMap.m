function Map = FuncInit3DNDTOccupancyMap(Pose,Scan,Param)

NumPose = size(Pose,1);
Origin = Param.Origin;

Scale = Param.Scale;

Size_i = Param.Size_i;
Size_j = Param.Size_j;
Size_h = Param.Size_h;

Grid = zeros(Size_i, Size_j, Size_h);
N = zeros(Size_i, Size_j, Size_h);

PointCloud = cell(1,NumPose);
% project 3D local observations into global coordinate 
for i = 1:NumPose

    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));

    Ri = FuncR(RZ',RY',RX');

    xyz = Scan{i}.xyz';
    Oddi = Scan{i}.Odd;

    HitId = find(Oddi>0);

    Si = Ri*xyz+Posei(1:3)'; % Point Cloud in Global Coordinate

    HitPoint = Si(:,HitId);
    
    PointCloud{i} = HitPoint';

    XYZ3 = (Si-Origin) / Scale + 1;
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));

    if sum((x<=0).*(y<=0).*(z<=0))~=0
        pfintf("Error Size \n\n");
    end

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;

    TemGrid = accumarray(ind',Oddi,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);
    TemGrid_reshaped = reshape(TemGrid, size(Grid));

    Grid = Grid + TemGrid_reshaped;

    TemN = accumarray(ind',1,[size(N,1)*size(N,2)*size(N,3),1]);
    TemN_reshaped = reshape(TemN, size(N));

    N = N + TemN_reshaped;

end

PointCloudAll = vertcat(PointCloud{:});

NDTMap = FuncCalNDTfromPointCloud(PointCloudAll,Size_i/5,Size_j/5,Size_h/5,Scale*5,Origin);

Map.Grid = Grid;
Map.N = N;
Map.Scale = Scale;
Map.Origin = Origin;
Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Size_h = Size_h;

% smooth OGM by NDT
ZoomScale = 5;
Map = FuncSmoothOGMbyNDT(Map,NDTMap,Scale,Origin,ZoomScale);



Dgrid = griddedInterpolant(Map.Grid);
Ngrid = griddedInterpolant(Map.N);



Map.DgridG = Dgrid;
Map.NgridG = Ngrid;

[Gdu,Gdv,Gdz] = gradient(Map.Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Gdzgrid = griddedInterpolant(Gdz);

Map.DgridGu = Gdugrid;
Map.DgridGv = Gdvgrid;
Map.DgridGz = Gdzgrid;

end
