function [states] = Traj_pol5_valid(initialStates, path_points, sv)

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
        %midpoint(4:7) = slerp(quaternion(path_points(segment,4:7)),quaternion(path_points(segment+1,4:7)),.5).compact;
        path_points = [path_points(1:segment,:); midpoint; path_points(segment+1:end,:)];

    end

    [ts, total_time] = Path_time(path_points);

    %states = Traj_pol5(path_points, total_time, ts);
    [m,n] = size(path); % n == 3
    tpts = 0:m;
    states = transpose(Traj_pol5(path_points, timepoints));

    % Check if the new trajectory is valid
    valid = all(isStateValid(sv,states));

end
end