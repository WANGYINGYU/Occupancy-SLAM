function Map = FuncUpdateMapNNorm(Map,Pose,Scan)

NumPose = size(Pose,1);

Scale = Map.Scale;
Origin = Map.Origin;

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

N = zeros(Size_i, Size_j, Size_h);
NumVoxel = Size_i * Size_j * Size_h;
% project 3D local observations into global coordinate 
for i = 1:NumPose

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

    % Per-pose binary hit occupancy, then accumulate across poses.
    TemN = accumarray(ind',1,[NumVoxel,1]);
    TemN(TemN > 1) = 1;
    N = N + reshape(TemN, size(N));

end

Map.N = N;

end
