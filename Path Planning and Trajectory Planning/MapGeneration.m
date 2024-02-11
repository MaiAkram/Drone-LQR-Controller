function [UAVmap3D, startPose, goalPose] = MapGeneration()
%rng(5,"twister");

% Creating a 3D empty map
UAVmap3D =  occupancyMap3D;
mapWidth = 200;
mapLength = 200;

% Adding obstacles to the map
numberOfObstacles = 15;
obstacleNumber = 1;
while obstacleNumber <= numberOfObstacles
    width = randi([10 25],1);                 % The largest integer in the sample intervals for obtaining width, length and height                                                     
    length = randi([10 25],1);                % can be changed as necessary to create different occupancy maps.
    height = randi([25 150],1);
    xPosition = randi([0 mapWidth-width],1);
    yPosition = randi([0 mapLength-length],1);
    
    [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition:xPosition+width,yPosition:yPosition+length,0:height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    
    checkIntersection = false;
    for i = 1:size(xyzObstacles,1)
        if checkOccupancy(UAVmap3D,xyzObstacles(i,:)) == 1
            checkIntersection = true;
            break
        end
    end
    if checkIntersection
        continue
    end
    
    setOccupancy(UAVmap3D,xyzObstacles,1)
    
    obstacleNumber = obstacleNumber + 1;
end

% Adding ground
[xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(UAVmap3D,xyzGround,1)

% Determining start and goal positions [x y z qw qx qy qz]
startPose = [StartGoal_allocation(UAVmap3D, [10 10 10], [20 20 20]) 0.7 0.2 0 0.1];
goalPose  = [StartGoal_allocation(UAVmap3D, [150 150 30], [200 200 35])  0.3 0 0.1 0.6];

% Plot the map
figure("Name","3D Map")
show(UAVmap3D)


