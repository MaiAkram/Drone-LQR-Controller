% Converting resulting quaternion states into euler angles phi, theta. psi

[yaw, pitch, roll] = quat2angle([States_AB(:,4) States_AB(:,5) States_AB(:,6) States_AB(:,7)]);
Rot_AB = [yaw, pitch, roll];
roll_AB = [time_AB, Rot_AB(:,1)];
pitch_AB = [time_AB, Rot_AB(:,2)];
yaw_AB = [time_AB, Rot_AB(:,3)];
Trans_AB = ([States_AB(:,1) States_AB(:,2) States_AB(:,3)]);
x_AB = [time_AB, Trans_AB(:,1)];
y_AB = [time_AB, Trans_AB(:,2)];
z_AB = [time_AB, Trans_AB(:,3)];

[yaw, pitch, roll] = quat2angle([States_BA(:,4) States_BA(:,5) States_BA(:,6) States_BA(:,7)]);
Rot_BA = [yaw, pitch, roll];
Trans_BA = ([States_BA(:,1) States_BA(:,2) States_BA(:,3)]);
