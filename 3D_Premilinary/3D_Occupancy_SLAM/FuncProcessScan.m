function NewScan = FuncProcessScan(Scan,Param)

Num = length(Scan);
NewScan = cell(1,Num);

NoiseLevel = 0.05;

for i=1:Num
    Scan_i = Scan{i}(2:end,:);

%     if Param.DownSampling
%         SampeScani = [];
%         for j=1:Param.SampleRate*360:size(Scan_i,2)
%             SampleScanij = Scan_i(:,j:(j+360));
%             SampeScani = [SampeScani,SampleScanij];
%         end
%         Scan_i = SampeScani;
%     end
    
    CheckNaN = sum(Scan_i,1);

    Id_Removed = find(isnan(CheckNaN)==1);

    Scan_i(:,Id_Removed) = [];

    % Add Noises

    NumPoints = size(Scan_i,2);
    
    NoiseXYZ = NoiseLevel * randn(3,NumPoints);
    UpperId = find(NoiseXYZ>2*NoiseLevel);
    NoiseXYZ(UpperId) = 2*NoiseLevel;

    LowerId = find(NoiseXYZ<-2*NoiseLevel);
    NoiseXYZ(LowerId) = -2*NoiseLevel;

    Scan_i = Scan_i(:,:) + NoiseXYZ;

    NewScan{i} = Scan_i;
end    

end