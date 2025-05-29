function SubMap = FuncInitLocalOccupancyMap(SubMapPose,SubMapScan,Param)

NumSubMap = size(SubMapScan,2);
SubMap = cell(1,NumSubMap);
LocalScale = Param.LocalScale;
OverlapScan = Param.OverlapScan;

for i=1:NumSubMap
    Scan = FuncEqualProcessObs(SubMapScan{i},Param);
    Pose  = SubMapPose{i};
    NumPose = size(Pose,1); 
    % Transfer All Poses to the Local Coordinates
    % Calculate the Relative Poses between Local and Global
    if i~=1
        NumOverlapScan = OverlapScan(i-1);
        FixFramePose = Pose(NumOverlapScan+1,:);
    else
        FixFramePose = Pose(1,:);
    end
    RelativeFixFrameTFMat = inv(FuncTFMatrix(FixFramePose));
    PoseLocal = zeros(NumPose,6);
    for j=1:NumPose
        PoseGlobali = Pose(j,:);
        TFMatGlobali = FuncTFMatrix(PoseGlobali);
        TFMatLocali = RelativeFixFrameTFMat * TFMatGlobali;
        [raw,pitch,yaw] = FuncRotMatrix2Euler(TFMatLocali(1:3,1:3));
        PoseLocali = [TFMatLocali(1:3,4)',raw,pitch,yaw];
        PoseLocal(j,:) = PoseLocali;
    end 
    Pose = PoseLocal; 
    PointsGlobal = [];
    parfor j = 1:NumPose
        Posei = Pose(j,:);
        RX = FuncRX(Posei(4));
        RY = FuncRY(Posei(5));
        RZ = FuncRZ(Posei(6));
        Ri = FuncR(RZ',RY',RX');
        xyz = double(Scan{j}.xyz');
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
    
    Origin = [int_minx;int_miny;int_minz];
    XYZ = (PointsGlobal-Origin);
    XYZ3 = XYZ/LocalScale+1;
    x = round(XYZ3(1,:));
    y = round(XYZ3(2,:));
    z = round(XYZ3(3,:));
    % calculate the map size
    max_x = fix((max(x)+1)/2)*2;
    max_y = fix((max(y)+1)/2)*2;
    max_z = fix((max(z)+1)/2)*2;
    
    Param.Origin = Origin;
    Size_i = max_y;
    Size_j = max_x;
    Size_h = max_z;
    Param.Size_i = Size_i;
    Param.Size_j = Size_j;
    Param.Size_h = Size_h;
    
    % calculate the map size
    Grid = zeros(Size_i, Size_j, Size_h);
    N = zeros(Size_i, Size_j, Size_h);

    for j=1:NumPose
        Posei = Pose(j,:);
        RX = FuncRX(Posei(4));
        RY = FuncRY(Posei(5));
        RZ = FuncRZ(Posei(6));
        Ri = FuncR(RZ',RY',RX');
        xyz = Scan{j}.xyz';
        Oddi = Scan{j}.Odd;
        Si = Ri*xyz+Posei(1:3)';
        XYZ = (Si-Origin);
        XYZ3 = [XYZ(1,:)/LocalScaleX;XYZ(2,:)/LocalScaleY;XYZ(3,:)/LocalScaleZ] + 1;
        x = round(XYZ3(1,:));
        y = round(XYZ3(2,:));
        z = round(XYZ3(3,:));
        Ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;
        TemN = accumarray(Ind',1,[size(N,1)*size(N,2)*size(N,3),1]);
        TemN_reshaped = reshape(TemN, size(N));
        N = N+TemN_reshaped;
        TemGrid = accumarray(Ind',Oddi,[size(Grid,1)*size(Grid,2)*size(Grid,3),1]);
        TemGrid_reshaped = reshape(TemGrid, size(Grid));
        Grid = Grid + TemGrid_reshaped;       
    end    
    clearvars Scan;
    Grid = Grid./N;
    Grid(isnan(Grid)) = 0;
    Grid(isinf(Grid)) = 0;
    N(N~=0) = 1;  
    Map.Scale = LocalScale;
    Map.Origin = Origin;
    Map.Size_i = Size_i;
    Map.Size_j = Size_j;
    Map.Size_h = Size_h;
    Map.Grid = Grid;
    Map.N = N;

    [Gdu,Gdv,Gdz] = gradient(Map.Grid);
    Gdugrid = griddedInterpolant(Gdu);
    Gdvgrid = griddedInterpolant(Gdv);
    Gdzgrid = griddedInterpolant(Gdz);
  
    Dgrid = griddedInterpolant(Map.Grid);
    Ngrid = griddedInterpolant(Map.N);
    Map.DgridG = Dgrid;
    Map.NgridG = Ngrid;
        
    Map.DgridGu = Gdugrid;
    Map.DgridGv = Gdvgrid;
    Map.DgridGz = Gdzgrid;
    SubMap{i} = Map;
end
end
