function FuncDrawTrajectory(Trajectory,TrajectoryGT,TrajectoryOdom,FigID)

figure(FigID)
plot3(Trajectory(:,1),Trajectory(:,2),Trajectory(:,3),'b-','LineWidth',2)
hold on
plot3(TrajectoryGT(:,1),TrajectoryGT(:,2),TrajectoryGT(:,3),'r-')
hold on
plot3(TrajectoryOdom(:,1),TrajectoryOdom(:,2),TrajectoryOdom(:,3),'g-')
hold off

end