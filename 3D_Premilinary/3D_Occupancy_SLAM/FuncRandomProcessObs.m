function Obs = FuncRandomProcessObs(Scan,Param)

NumPose = length(Scan);
MaxRange = Param.MaxRange;
MinRange = Param.Scale;

SampleDistance = Param.Scale;

% ValOddHit = Param.ValOddHit;
% ValOddFree = Param.ValOddHit;

Obs = cell(1,NumPose);

for i=1:NumPose
    HitPi = Scan{i}(1:end,:);
    CheckSum = sum(HitPi,1);
    IdNaN = isnan(CheckSum)==1;
    HitPi(:,IdNaN) = [];

    % Check Distance
    Distance = sqrt(HitPi(1,:).^2 + HitPi(2,:).^2 + HitPi(3,:).^2);
    OutId = find((Distance>=MaxRange).*(Distance<=MinRange)~=0);
    HitPi(:,OutId) = [];
    
    [AllXYZi,AllOddi] = FuncRandomSample3DLines(HitPi',5,Param);

    Obs{i}.xyz = AllXYZi;
    Obs{i}.Odd = AllOddi;
end

end