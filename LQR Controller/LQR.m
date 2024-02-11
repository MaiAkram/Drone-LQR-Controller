A = [0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 g 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 g 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 l/Ixx 0 0;
     0 0 0 0;
     0 0 l/Iyy 0;
     0 0 0 0;
     0 0 0 1/Izz];
% C = [1 0 0 0 0 0 0 0 0 0 0 0;
%      0 0 1 0 0 0 0 0 0 0 0 0;
%      0 0 0 0 1 0 0 0 0 0 0 0;
%      0 0 0 0 0 0 1 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 1 0 0 0;
%      0 0 0 0 0 0 0 0 0 0 1 0];
C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0];
%D = 0;
D = zeros(4,4);

% CONTROLLABILITY CHECK
r_c = rank(ctrb(A,B));

% OBSERVABILITY CHECK
r_o = rank(obsv(A,C));

% LQR CONTROLLER
Q = transpose(C)*C;
Q(11,11)=10;
%Q = diag([2 1 2 1 1 1 15 10 15 10 10 1]);
R = diag([1 1 1 1]);
% R = [1 0 0 0;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];

[K,P,E] = lqr(A,B,Q,R);

k1 = K(3,1);
k2 = K(3,2);
k3 = K(2,3);
k4 = K(2,4);
k5 = K(1,5);
k6 = K(1,6);
k7 = K(2,7);
k8 = K(2,8);
k9 = K(3,9);
k10 = K(3,10);
k11 = K(4,11);
k12 = K(4,12);

% LQI STATE SPACE MATRICES
na = zeros(12,4);
nc = zeros(4,4);
A_i = [A na; -C nc];
B_i = [B; nc];
C_i = [C nc];

% CONTROLLABILITY CHECK
r_ci = rank(ctrb(A_i,B_i));

% OBSERVABILITY CHECK
r_oi = rank(obsv(A_i,C_i));

% LQI CONTROLLER

%Q_i = transpose(C_i)*C_i;
Q_i = diag([2 1 2 1 1 1 15 10 15 10 10 1 5 5 5 5]);
R_i = diag([1 1 1 1]);
% R = [1 0 0 0;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];

[Ki,Pi,Ei] = lqr(A_i,B_i,Q_i,R_i);

ki1 = Ki(3,1);
ki2 = Ki(3,2);
ki3 = Ki(2,3);
ki4 = Ki(2,4);
ki5 = Ki(1,5);
ki6 = Ki(1,6);
ki7 = Ki(2,7);
ki8 = Ki(2,8);
ki9 = Ki(3,9);
ki10 = Ki(3,10);
ki11 = Ki(4,11);
ki12 = Ki(4,12);
ki13 = Ki(3,13);
ki14 = Ki(2,14);
ki15 = Ki(1,15);
ki16 = Ki(4,16);


% NEW STATE SPACE MATRICES

AA = A - B*K;
BB = B*k1;
t = 0:0.01:8;

f = step(AA,BB,C,D,1,t);
% plot(f)
x = f(:,1);
y = f(:,2);
z = f(:,3);
psi = f(:,4);
% plot(x)
% plot(y)
% plot(z)
% plot(psi)
