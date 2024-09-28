function Pose = FuncUpdatePose(DeltaP,Pose)

DeltaP2 = reshape(DeltaP,6,[])';
Pose(2:end,:) = Pose(2:end,:)+DeltaP2;

end
