function Obs = FuncProcessNearObs(Scan,SampleDistance,Pose)

NumPose = length(Scan);
Obs = cell(1,NumPose);

ValOddHit = 0.847297860387203;
ValOddFree = -0.405465108108164;
HyperMaxNum = 10;

Obs = cell(1,NumPose);

for i=1:NumPose
    HitPi = Scan{i}(2:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];
    HitObs{1,i} = HitPi;

    % Posei = Pose(i,:);
    % RX = FuncRX(Posei(4));
    % RY = FuncRY(Posei(5));
    % RZ = FuncRZ(Posei(6));
    % 
    % Ri = FuncR(RZ',RY',RX');
    % Si = Ri*HitPi+Posei(1:3)';
    % 
    % IdDelete = find(Si(2,:)>=2);
    % HitPi(:,IdDelete) = [];

    [Azimuthi,Elevationi,Rangei] = cart2sph(HitPi(1,:),HitPi(2,:),HitPi(3,:));
    
    AllXYZi = [];
    AllOddi = [];
    
    for j=1:length(Rangei)
        Rangeij = Rangei(j);
        MaxNum = fix(Rangeij/SampleDistance);
        MaxNum = min(HyperMaxNum,MaxNum);
        LastRange = Rangeij - MaxNum * SampleDistance;
        AllRangeij = LastRange:SampleDistance:Rangeij;
        Azimuthij = repmat(Azimuthi(j),1,length(AllRangeij));
        Elevationij = repmat(Elevationi(j),1,length(AllRangeij));
        [AllXij,AllYij,AllZij] = sph2cart(Azimuthij,Elevationij,AllRangeij);

        % IdDelete = find(AllYij>=2);

        AllXYZij = [AllXij;AllYij;AllZij];

        % AllXYZij(:,IdDelete) = [];

        AllXYZi = [AllXYZi;AllXYZij'];
        AllOddij = repmat(ValOddFree,1,length(AllRangeij));
        AllOddij(end) = ValOddHit;
        AllOddi = [AllOddi;AllOddij'];
    end 

    Obs{i}.xyz = AllXYZi;
    Obs{i}.Odd = AllOddi;
end

end