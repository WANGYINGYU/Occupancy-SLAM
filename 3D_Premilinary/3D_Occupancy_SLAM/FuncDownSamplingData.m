function [DownPose, DownScan] = FuncDownSamplingData(Pose,Scan,Rate)

k = 1;
Num = length(Scan);
DownNum = fix(Num./Rate);
DownPose = zeros(DownNum,6);
DownScan = cell(1,DownNum);

for i=1:Rate:length(Scan)
    DownPose(k,:) = Pose(i,:);
    DownScan(k) = Scan(i);
    k = k+1;
end