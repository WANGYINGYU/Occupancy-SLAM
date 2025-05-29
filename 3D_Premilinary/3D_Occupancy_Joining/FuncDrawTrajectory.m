function FuncDrawTrajectory(Trajectory,TrajectoryGT,FigID)

figure(FigID)
plot3(Trajectory(:,1),Trajectory(:,2),Trajectory(:,3),'b-','LineWidth',2)
% axis([-40,80,-30,90])
hold on
plot3(TrajectoryGT(:,1),TrajectoryGT(:,2),TrajectoryGT(:,3),'r-')
hold off

end