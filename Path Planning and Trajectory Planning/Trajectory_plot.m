function [] = Trajectory_plot(omap, startPose, goalPose, pthObj, initialStates, states)

% Plot map, start and goal positions
show(omap)
hold on
scatter3(startPose(1),startPose(2),startPose(3),30,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),30,".g")

% Plot the planned path points
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")

% Plot the quintic polynomial initial trajectory
plot3(initialStates(:,1),initialStates(:,2),initialStates(:,3),"-y")

% Plot the quintic polynomial valid trajectory
plot3(states(:,1),states(:,2),states(:,3),"-c")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path","Initial Trajectory","Valid Trajectory")
hold off