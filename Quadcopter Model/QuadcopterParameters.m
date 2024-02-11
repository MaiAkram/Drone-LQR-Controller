%ABGULAR VELOCITIES
w1 = 200;
w2 = 200;
w3 = 10;
w4 = 10;

% MOMENTS OF INERTIA
Ixx = 7.5e-3;
Iyy = 7.5e-3;
Izz = 1.3e-2;

% GYRO
wr = 0;
Jr = 0;

% LENGTH
l = 0.23;

% MASS & GRAVITATIONAL ACCELERATION
m = 0.65;
%m = 1;
g = 9.81;

% COEFFITIENTS
Kf = 3.13e-5;
Km = 7.5e-7;

% PID
Kp_z = 10;
Ki_z = 1;
Kd_z = 0.5;

Kp_psi = 5;
Ki_psi = 3;
Kd_psi = 0.5;

Kp_x = 0;
Ki_x = -1;
Kd_x = 0.5;

Kp_t = 0;
Ki_t = -1;
Kd_t = 0.5;
