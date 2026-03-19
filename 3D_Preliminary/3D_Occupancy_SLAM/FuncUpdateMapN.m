function Map = FuncUpdateMapN(Map,Pose,Scan)

NumPose = size(Pose,1);

Scale = Map.Scale;
Origin = Map.Origin;

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

numVoxels = Size_i * Size_j * Size_h;
NLin = zeros(numVoxels,1);
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

    if sum((x<=0).*(y<=0).*(z<=0))~=0
        pfintf("Error Size \n\n");
    end

    ind = (Size_i*Size_j*(z-1)) + (x-1)*Size_i + y;

    TemN = accumarray(ind',1,[numVoxels,1]);

    IdMore = TemN > 1;

    TemN(IdMore) = 1;

    NLin = NLin + TemN;

end

N = reshape(NLin, [Size_i, Size_j, Size_h]);

Map.N = N;

end
