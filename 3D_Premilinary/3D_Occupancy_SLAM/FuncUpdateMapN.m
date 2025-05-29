function Map = FuncUpdateMapN(Map,Pose,Scan)

NumPose = size(Pose,1);

Scale = Map.Scale;
Origin = Map.Origin;

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

N = zeros(Size_i, Size_j, Size_h);
cell_N = cell(1,NumPose);
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

    TemN = accumarray(ind',1,[size(N,1)*size(N,2)*size(N,3),1]);

    IdMore = find(TemN>1);

    TemN(IdMore) = 1;

    TemN_reshaped = reshape(TemN, size(N));

    cell_N{1,i} = TemN_reshaped; 

end

N = sum(cat(4, cell_N{:}), 4);

Map.N = N;

end
