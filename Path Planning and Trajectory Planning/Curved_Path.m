% % UAV Path and Trajectory Planning From Point A to B and back to A
% 
% % Load Map
% map =  occupancyMap3D;
% [xObstacle,yObstacle,zObstacle] = meshgrid(10:10+20,20:20+25,0:30);
% setOccupancy(UAVmap3D,xyzObstacles,1);
% % Adding ground
% [xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
% xyzGround = [xGround(:) yGround(:) zGround(:)];
% setOccupancy(map,xyzGround,1)
% % Plot the map
% figure("Name","3D Map")
% show(map)
% 
% % Consider unknown spaces to be unoccupied
% omap = map;
% omap.FreeThreshold = omap.OccupiedThreshold;
% show(omap)
% 
% % Showing allocated start and goal positions
% A = [StartGoal_allocation(omap, [5 15 5], [8 18 15]) 0.7 0.2 0 0.1];
% B = [StartGoal_allocation(omap, [32 47 20], [35 50 30])  0.3 0 0.1 0.6];
% 
% figure
% [States_AB, time_AB] = RRT_planner(UAVmap3D, A, B);
% time_AB = transpose(time_AB);
% figure
% [States_BA, time_BA] = RRT_planner(UAVmap3D, B, A);
% time_BA = transpose(time_BA);
% 
% % UAV Path and Trajectory Planning From Point A to B and back to A
% % Showing allocated start and goal positions
% A = [StartGoal_allocation(omap, [5 15 5], [8 18 15]) 0.7 0.2 0 0.1];
% B = [StartGoal_allocation(omap, [32 47 20], [35 50 30])  0.3 0 0.1 0.6];
% 
% figure
% [States_AB, time_AB] = RRT_planner(UAVmap3D, A, B);
% time_AB = transpose(time_AB);
% figure
% [States_BA, time_BA] = RRT_planner(UAVmap3D, B, A);
% time_BA = transpose(time_BA);
% 
% % Load Map
% map =  occupancyMap3D;
% [xObstacle,yObstacle,zObstacle] = meshgrid(10:10+20,20:20+25,0:30);
% setOccupancy(UAVmap3D,xyzObstacles,1);
% % Adding ground
% [xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
% xyzGround = [xGround(:) yGround(:) zGround(:)];
% setOccupancy(map,xyzGround,1)
% % Plot the map
% figure("Name","3D Map")
% show(map)
% 
% % Consider unknown spaces to be unoccupied
% omap = map;
% omap.FreeThreshold = omap.OccupiedThreshold;
% show(omap)

% Sample time
ts = 0.01;
g = [0;0;9.8];

% Initial States
euler_0 = [0;0;0];
xyz_0 = [0;0;0];
rates_0 = [0;0;0];

% Motion Time
t_f = 500;
tss = 0.1;
time = 0:tss:t_f;

% Motion Equaation
x = 2*sin(0.3*time);
y = 2 - 2*cos(0.3*time);
z = 3 + sin(0.6*time);
x = sin(0.1*t);
y = cos(0.1*t);
z = 0.3*t;
% x = sin(0.1*t);
% y = cos(0.1*t);
% z = 4;

path = zeros(length(time),3);
for i = 1:length(x)
    path(i,1) = x(i);
    path(i,2) = y(i);
    path(i,3) = z(i);
end

% Signal
takeOffT = 1;
flyT = 2;
fT = flyT + tss * (length(path)-1);
t = 0:tss:fT;

% Trajectory Planning
[q, qd, qdd, pp] = Traj_pol5(path, time);

% Animation
figure
x_l = [-5,5];
y_l = [-5,5];
z_l = [0,10];
width = 750;
height = 650;
%figure(x_l,y_l,z_l,-37,17,width,height);
pause(1);
%AnimEulerTar(out.time,out.XYZ,out.EulerAngles,out.VXYZ)

UAVmap3D =  occupancyMap3D;
% Adding ground
[xGround,yGround,zGround] = meshgrid(-5:5,-5:5,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(UAVmap3D,xyzGround,1)

show(UAVmap3D)
hold on
scatter3(q(1,1),q(2,1),q(3,1),30,".r")
scatter3(q(1,251),q(2,251),q(3,251),30,".b")

q_ = transpose(q);
% Plot the planned path points
plot3(q_(:,1),q_(:,2),q_(:,3),"-g")


