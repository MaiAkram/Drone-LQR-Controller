function [path_points, states, timepoints] = RRT_planner(omap, startPose, goalPose)
% RRT* (RAPIDLY-EXPLORING RANDOM TREE) PATH PLANNER

% Plot the start and goal poses
hold on
scatter3(startPose(1),startPose(2),startPose(3),100,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),100,".g")
view([-31 63])
legend("Start Position","Goal Position")
hold off

% Define the state space limits [x y z qw qx qy qz] 
% Distance in meters and Orientation in quaternion representaion
ss = stateSpaceSE3([-20 220;
                    -20 220;
                    -10 100;
                    inf inf;
                    inf inf;
                    inf inf;
                    inf inf]);

% Defining states validity according to present obstacles
sv = validatorOccupancyMap3D(ss,Map=omap);
sv.ValidationDistance = 0.1;

% Setting planner criteria
planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance = 1;
planner.GoalBias = 0.8;
planner.MaxIterations = 10000;
planner.ContinueAfterGoalReached = true;
planner.MaxNumTreeNodes = 100000;

% Applying the RRT* planner to obtain the UAV path
[pthObj,solnInfo] = plan(planner,startPose,goalPose);

% Exiting the program if no path is found
if (~solnInfo.IsPathFound)
    disp("RRT* Planner Found NO Path")
    return
end

% Plot map, start pose, and goal pose
show(omap)
hold on
scatter3(startPose(1),startPose(2),startPose(3),100,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),100,".g")

% Plot the planned path
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path")
hold off

% Storing path points and number
path_points = pthObj.States;
nWayPoints = pthObj.NumStates;

% Calculate the distance between path points
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
    distance(i) = norm(path_points(i,1:3) - path_points(i-1,1:3));
end

% Assume a UAV speed of 3 m/s and calculate time taken to reach each waypoint
UAVspeed = 3;
timepoints = cumsum(distance/UAVspeed);
%nSamples = 100;

% Function for measuring total path time and points in-between time
%[ts, total_time] = Path_time(path_points);

% Perfoemong trajectory planning of the generated path
%initialStates = Traj_pol5(path_points, total_time, ts);
initialStates = transpose(Traj_pol5(path_points, timepoints));

% Enhancing the path for obstacel avoidance
states = Traj_pol5_valid(initialStates, path_points, sv);

% Plotting the generated path
Trajectory_plot(omap, startPose, goalPose, pthObj, initialStates, states)
