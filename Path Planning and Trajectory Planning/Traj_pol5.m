function [q, qd, qdd, pp] = Traj_pol5(path, timepoints) %Traj_pol5(path, total_time, ts)

[m,n] = size(path); % n == 3
%m = m-1;

wpts = transpose(path);
tpts = 1:m;
%tvec = 0:total_time:ts;
tvect = timepoints;

[q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tvect);

% figure
% plot(q(1,:),q(2,:), q(3,:),'.b',wpts(1,:),wpts(2,:),wpts(3,:),'or')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')


% eulerAngles = [0 0 0; 0 0 0; 180 0 0; 180 0 0; 0 0 0];
% quat = quaternion(eulerAngles,"eulerd","ZYX","frame");
% trajectory = polynomialTrajectory(pp,SampleRate=100,Orientation=quat);
% waypointInfo(traj);
% 
% [pos,orient,vel,acc,angvel] = trajectory();
% 
% eulOrientation = quat2eul(orient);
% yawAngle = eulOrientation(:,1);
% 
% 
% 
% i = 1;
% spf = traj.SamplesPerFrame;
% while ~isDone(traj)
%     idx = (i+1):(i+spf);
%     [q(idx,:),orient(idx,:), ...
%      vel(idx,:),acc(idx,:),angvel(idx,:)] = trajectory();
%     i = i + spf;
% end
% 
% plot3(pos(:,1),pos(:,2),pos(:,3), ...
%       path(1,:),path(2,:),path(3,:),"--o")
% hold on
% % Plot the yaw using quiver.
% quiverIdx = 1:100:length(pos);
% quiver3(pos(quiverIdx,1),pos(quiverIdx,2),pos(quiverIdx,3), ...
%         cos(yawAngle(quiverIdx)),sin(yawAngle(quiverIdx)), ...
%         zeros(numel(quiverIdx),1))
% title("Position")
% xlabel("X (m)")
% ylabel("Y (m)")
% zlabel("Z (m)")
% legend({"Position","Path","Orientation"})
% axis equal
% hold off

