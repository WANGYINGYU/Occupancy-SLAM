function Pose = FuncLeastSquaresBresenham(Map,Pose,PoseGT,WorldObs,OriginalScan,Param,FILE_DIRECTORY,HH,Odom)

Weight = 0.00000000000001;
HH2 = Weight*HH;
Scale = Param.Scale;
MaxIter = 100;

figure(1)
ab = Map.Grid(:,:,8);
ab = exp(ab);
ShowMap = ab./(1+ab);
imshow(1-ShowMap)

MAETran = mean(abs(PoseGT(:,1:3) - Pose(:,1:3)))
MAEAng = mean(abs(PoseGT(:,4:6) - Pose(:,4:6)))

for i=1:MaxIter


    [ErrorS,ErrorO,Mean_Error,JP,JM,JO] = FuncDiffJacobianBresenham(Map,Pose,WorldObs,Odom);

    % [ErrorS,Mean_Error,JP,JM] = FuncDiffJacobianFix(Map,Pose,Scan);

    % [DeltaP,DeltaM,Mean_Delta,Mean_Delta_Pose] = FuncDelta(JP,JM,JO,ErrorS,ErrorO,Map,HH2);
    [DeltaP,DeltaM,Mean_Delta,Mean_Delta_Pose] = FuncFix2DDelta(JP,JM,ErrorS,Map,HH2);
    [Map,Pose] = FuncUpdate3D(Map,Pose,DeltaP,DeltaM);
    WorldObs = FuncReProcessObsBresenham(OriginalScan,Pose,Param);

    % Map = FuncUpdateMapN(Map,Pose,Scan);
    Map = FuncUpdateMapNBresenham(Map,WorldObs);

    figure(3)
    ab = Map.Grid(:,:,5);
    ab = exp(ab);
    ShowMap = ab./(1+ab);
    imshow(1-ShowMap)

    % Map = FuncInit3DOccupancyMap(Pose,Scan,Param);

    % Map = FuncInit3DNDTOccupancyMap(Pose,Scan,Param);


    MAETran = mean(mean(abs(PoseGT(:,1:3) - Pose(:,1:3))));
    MAEAng = mean(mean(abs(PoseGT(:,4:6) - Pose(:,4:6))));

    txt=strcat(FILE_DIRECTORY,'/result.txt');
    fid = fopen(txt,'a');

    fprintf("Iteration Time is %i\n\nMean Translation Error is %4f, Mean Rotation Error is %4f, Mean Observation Error is %4f\n\n", i,MAETran,MAEAng,Mean_Error);
    fprintf(fid,"Iteration Time is %i\n\nMean Translation Error is %4f, Mean Rotation Error is %4f, Mean Observation Error is %4f\n\n", i,MAETran,MAEAng,Mean_Error);


    MAETran = mean(abs(PoseGT(:,1:3) - Pose(:,1:3)))
    MAEAng = mean(abs(PoseGT(:,4:6) - Pose(:,4:6)))

    % fprintf("Translation X Error is %4f, Translation Y Error is %4f, Translation Z Error is %4f, Rotation r Error is %4f, Rotation p Error is %4f, Rotation y Error is %4f", MAETran(1),MAETran(2),MAETran(3),MAEAng(1),MAEAng(2),MAEAng(3));
    fprintf(fid,"Translation  Error is %4f, %4f, %4f, Rotation Error is %4f, %4f, %4f", MAETran(1),MAETran(2),MAETran(3),MAEAng(1),MAEAng(2),MAEAng(3));

    fclose(fid);

     
    % Map.Grid = smooth3(Map.Grid,'gaussian',3);
    % Map.N = smooth3(Map.N,'gaussian',3);


    if mod(i,30)==0
        % Weight = Weight/100;
        Param.Scale = Param.Scale/2;
        Map = FuncInit3DOccupancyMapNorm(Pose,Scan,Param);
        HH = FuncMapConst3D(Map);
        HH2 = Weight*HH;
    end    


    Map = FuncSmoothN2(Map,Weight,HH2);

    Map = FuncMapGrid(Map);
    
    % FuncShow3DOccupancyMap(Map.Grid);

    Quat = angle2quat(Pose(:,4), Pose(:,5), Pose(:,6), 'XYZ');
    PoseQuat = [Pose(:,1:3),Quat];

    OccupancyMap = occupancyMap3D(5,"ProbabilitySaturation",[0.001,0.999]);

    for j = 1:length(OriginalScan)
        HitPi = OriginalScan{j}(1:end,:);
        CheckSum = sum(HitPi,1);
        IdNaN = isnan(CheckSum)==1;
        HitPi(:,IdNaN) = [];
        insertPointCloud(OccupancyMap,PoseQuat(j,:),HitPi',30);
    end
    figure(2)
    show(OccupancyMap)


end    


end