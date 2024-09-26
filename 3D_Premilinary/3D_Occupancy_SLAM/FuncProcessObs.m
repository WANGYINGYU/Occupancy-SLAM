function Obs = FuncProcessObs(Scan,Param)

NumPose = length(Scan);
MaxRange = Param.MaxRange;
MinRange = Param.Scale;

SampleDistance = Param.Scale;

ValOddHit = Param.ValOddHit;
ValOddFree = Param.ValOddFree;

Obs = cell(1,NumPose);

parfor i=1:NumPose
    HitPi = Scan{i}(1:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];

    % Check Distance
    Distance = sqrt(HitPi(1,:).^2 + HitPi(2,:).^2 + HitPi(3,:).^2);
    OutId = find((Distance>=MaxRange)|(Distance<=MinRange));
    HitPi(:,OutId) = [];

    [Azimuthi,Elevationi,Rangei] = cart2sph(HitPi(1,:),HitPi(2,:),HitPi(3,:));
    
    AllXYZi = [];
    AllOddi = [];
    for j=1:length(Rangei)
        Rangeij = Rangei(j);
        MaxNum = fix(Rangeij/SampleDistance);
        MaxNum = min(MaxNum,fix(10/SampleDistance));
        LastRange = Rangeij - MaxNum * SampleDistance;
        AllRangeij = LastRange:SampleDistance:Rangeij;
        Azimuthij = repmat(Azimuthi(j),1,length(AllRangeij));
        Elevationij = repmat(Elevationi(j),1,length(AllRangeij));
        [AllXij,AllYij,AllZij] = sph2cart(Azimuthij,Elevationij,AllRangeij);
        AllXYZij = [AllXij;AllYij;AllZij];
        AllXYZi = [AllXYZi;AllXYZij'];
        AllOddij = repmat(ValOddFree,1,length(AllRangeij));
        AllOddij(end) = ValOddHit;
        AllOddi = [AllOddi;AllOddij'];
    end 

    Obs{i}.xyz = AllXYZi;
    Obs{i}.Odd = AllOddi;
end

end