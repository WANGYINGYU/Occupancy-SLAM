function TFMatrix = FuncPoseQuat2TFMatrix(PoseQuat)

XYZ = PoseQuat(1:3);

Quat = PoseQuat(4:7);

R = quat2rotm(Quat);


TFMatrix = [R, XYZ';
     0, 0, 0, 1];



end