function Obs = FuncProcessObs(Scan,Param)

NumPose = length(Scan);
MaxRange = Param.MaxRange;
MinRange = Param.LocalScale;

SampleDistance = Param.LocalScale;

ValOddHit = Param.ValOddHit;
ValOddFree = Param.ValOddFree;

Obs = cell(1,NumPose);

parfor i=1:NumPose
    HitPi = Scan{i}(1:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];

    [Azimuthi,Elevationi,Rangei] = cart2sph(HitPi(1,:),HitPi(2,:),HitPi(3,:));
    % Find Range out of MaxRange or MinRange
    IdOut = find(Rangei>MaxRange|Rangei<MinRange);
    Azimuthi(IdOut) = [];
    Elevationi(IdOut) = [];
    Rangei(IdOut) = [];
    AllXYZi = [];
    AllOddi = [];
    for j=1:length(Rangei)
        Rangeij = Rangei(j);
        MaxNum = fix(Rangeij/SampleDistance);
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