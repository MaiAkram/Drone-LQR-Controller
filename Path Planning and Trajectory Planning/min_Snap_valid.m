function [states] = min_Snap_valid(initialStates, path_points, sv)

% Check if the trajectory is valid
states = initialStates;
valid = all(isStateValid(sv,states));

while(~valid)
    % Check the validity of the states
    validity = isStateValid(sv,states);

    % Map the states to the corresponding waypoint segments
    segmentIndices = exampleHelperMapStatesToPathSegments(path_points,states);

    % Get the segments for the invalid states
    % Use unique, because multiple states in the same segment might be invalid
    invalidSegments = unique(segmentIndices(~validity));

    % Add intermediate waypoints on the invalid segments
    for i = 1:size(invalidSegments)
        segment = invalidSegments(i);
        
        % Take the midpoint of the position to get the intermediate position
        midpoint(1:3) = (path_points(segment,1:3) + path_points(segment+1,1:3))/2;
        
        % Spherically interpolate the quaternions to get the intermediate quaternion
        midpoint(4:7) = slerp(quaternion(path_points(segment,4:7)),quaternion(path_points(segment+1,4:7)),.5).compact;
        path_points = [path_points(1:segment,:); midpoint; path_points(segment+1:end,:)];

    end

    nWayPoints = size(path_points,1);
    distance = zeros(1,nWayPoints);
    for i = 2:nWayPoints
        distance(i) = norm(path_points(i,1:3) - path_points(i-1,1:3));
    end
    
    % Calculate the time taken to reach each waypoint
    timepoints = cumsum(distance/UAVspeed);
    nSamples = 100;
    states = minsnappolytraj(path_points',timepoints,nSamples,MinSegmentTime=2,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)';    
    
    % Check if the new trajectory is valid
    valid = all(isStateValid(sv,states));
   
end