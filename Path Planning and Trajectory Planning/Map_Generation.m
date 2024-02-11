% Creating a 3D empty map
UAVmap3D =  occupancyMap3D;
mapWidth = 100;
mapLength = 100;

% Adding obstacles to the map
numberOfObstacles = 15;
obstacleNumber = 1;
while obstacleNumber <= numberOfObstacles
    width = randi([5 10],1);                 % The largest integer in the sample intervals for obtaining width, length and height                                                     
    length = randi([5 10],1);                % can be changed as necessary to create different occupancy maps.
    height = randi([25 75],1);
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

% Plot the map
figure("Name","3D Map")
show(UAVmap3D)
