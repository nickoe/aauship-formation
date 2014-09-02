% Thrust allocation 
clear all;

% Arm
r(1:3,1) = [0.41;0.00;-0.05];
r(1:3,2) = [-0.18;0.00;-0.05];
r(1:3,3) = [-0.48;0.05;-5.05];
r(1:3,4) = [-0.48;-0.05;-5.05];

alpha = atan(r(3,:)./r(1,:))

F1 = [0;1;0]
F2 = [0;1;0]
F3 = [cos(alpha(3));0;sin(alpha(3))]
F4 = [cos(alpha(4));0;sin(alpha(4))]

F1 = cross(r(:,1),F1)
F2 = cross(r(:,2),F2)
F3 = cross(r(:,3),F3)
F4 = cross(r(:,4),F4)

T = [F1,F2,F3,F4] % Torque
T = [0,0,1,1;1,1,0,0;T]
% 
% T = [ 0 0 1 1;...
%       1 1 -sin(a) sin(a);...
%       -1 -1 0 0;...
%       0 0 sin(az)*lz3 sin(az*lz4);...
%       lx1 -lx2 -sin(a)*lx3 sin(a)*lx4];
K = eye(4,4);
% K(3,3) = 0.2657/2;
% K(4,4) = 0.2657/2;
u = [10 10 0 0]'; % Thruster force vector [N]
tau = T*K*u
% ta = T*K*(u+[0 0 24.8350/2 24.8350/2]')