function FuncEvaluatePose(Pose,PoseGT,Param)

MAETran = mean(mean(abs(PoseGT(:,1:3) - Pose(:,1:3))));
MAEAng = mean(mean(abs(PoseGT(:,4:6) - Pose(:,4:6))));
fprintf("Mean Translation Error is %4f\n\nMean Rotation Error is %4f\n\n", MAETran,MAEAng);

txt=strcat(Param.FileDict,'/result.txt');
fid = fopen(txt,'a');
fprintf(fid,"Mean Translation Error is %4f\n\nMean Translation Error is %4f\n\n", MAETran,MAEAng);
fclose(fid);

end