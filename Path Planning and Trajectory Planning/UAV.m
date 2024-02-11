% UAV Path and Trajectory Planning From Point A to B and back to A

% Load Map
omap = UAVmap3D;

% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
show(omap)

% Showing allocated start and goal positions
A = [StartGoal_allocation(omap, [10 10 10], [20 20 20]) 0.7 0.2 0 0.1];
%B = [StartGoal_allocation(omap, [150 150 50], [200 200 100])  0.3 0 0.1 0.6];
B = [StartGoal_allocation(omap, [50 50 50], [70 70 60])  0.3 0 0.1 0.6];

figure
[path_points_AB, States_AB, time_AB] = RRT_planner(UAVmap3D, A, B);
time_AB = transpose(time_AB);
figure
[path_points_BA, States_BA, time_BA] = RRT_planner(UAVmap3D, B, A);
time_BA = transpose(time_BA);

