function Param = FuncCalOriginAndMapSize(Pose,Scan,Param)

NumPose = size(Pose,1);
Scale = Param.Scale;

PointsGlobal = [];
for i = 1:NumPose
    Posei = Pose(i,:);
    RX = FuncRX(Posei(4));
    RY = FuncRY(Posei(5));
    RZ = FuncRZ(Posei(6));
    Ri = FuncR(RZ',RY',RX');
    xyz = double(Scan{i});
    Si = Ri*xyz+Posei(1:3)';
    Si = [Si,Posei(1:3)'];
    PointsGlobal = [PointsGlobal,Si];
end

% find the origin and size of map
min_x = min(PointsGlobal(1,:));
min_y = min(PointsGlobal(2,:));
min_z = min(PointsGlobal(3,:));

if min_x <=1e-6
    int_minx = floor(min_x)-3;
else
    int_minx = fix(min_x)-3;
end

if min_y <=1e-6
    int_miny = floor(min_y)-3;
else
    int_miny = fix(min_y)-3;
end

if min_z <=1e-6
    int_minz = floor(min_z)-3;
else
    int_minz = fix(min_z)-3;
end

Origin = [int_minx;int_miny;int_minz];

XYZ3 = (PointsGlobal-Origin) / Scale + 1;

x = round(XYZ3(1,:));
y = round(XYZ3(2,:));
z = round(XYZ3(3,:));

% calculate the map size
max_x = max(x);
max_y = max(y);
max_z = max(z);

Param.Origin = Origin;
Size_i = max_y+3/Scale;
Size_j = max_x+3/Scale;
Size_h = max_z+3/Scale;

Param.Size_i = Size_i;
Param.Size_j = Size_j;
Param.Size_h = Size_h;