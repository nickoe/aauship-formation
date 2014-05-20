%% Simulation of AAUSHIP in open loop

clear all
r_g = [0.03 0 0.03]'; % location of CG with respect to CO
% r_g = [-0.46 0 -3.54]'; % TP-MB-shipmod.pdf container ship
I_g = [ 6.5406e-02  -1.2600e-02  -5.3593e-02
       -1.2600e-02   1.0892e+00  -1.0800e-03
       -5.3593e-02  -1.0800e-03   1.1068e+00]; % inertia tensor, calculated in inertia.m
%    I_g = [ 6.5406e-02  0 0
%        0   1.0892e+00  0
%        0 0   1.1068e+00];
m   = 13;             % mass

% rigid body mass matrix, page 53 fossen
MRB = [ m*eye(3)     -m*Smtrx(r_g)
        m*Smtrx(r_g) I_g          ];
MRB = [MRB(1:2,1:2) MRB(1:2,4:6)
       MRB(4:6,1:2) MRB(4:6,4:6)]; % 5DOF, without heave
    
 
% CRB = m2c(MRB,nu) % rigid body Coriolis matrix, page 56 fossen

% linear hydrodynamic damping
Xu =  2.86;
Yv = 32.5;
Yp = -0.00503;
Yr =  0.09263;
Kv = -0.975;
Kp =  0.00084;
Kr =  0.01273;
Mq =  0.0712;
Nv =  0.975;
Np = -0.00069;
Nr =  0.26285;

% Data from TP-MB-shipmod.pdf
% Xu = -226.2;
% Yv = -725.0;
% Yp = -3.4;
% Yr = 118.2;
% Kv = 25.0;
% Kp = -3.0;
% Kr = 0.8;
% Mq = -1;
% Nv = -300;
% Np = -8.0;
% Nr = -290;

D = [Xu  0  0  0  0
       0 Yv Yp  0 Yr
       0 Kv Kp  0 Kr
       0  0  0 Mq  0
       0 Nv Np  0 Nr];


% linear restoring forces matrix
G = zeros(5,5);
K_phi   = 0.0007;
M_theta = 0.0150;
G(3,3) = K_phi;
G(4,4) = M_theta;

% Input forces
tau = [ 0 0 0 0 0]';

% Initial states
x0 = [0     % Surge pos (x)
      0     % Sway pos (y)
      0     % Roll pos (phi)
      0     % Pitch pos (theta)
      0     % Yaw pos (psi)
      1     % Surge vel (u)
      0     % Sway vel (v)
      1     % Roll vel (p)
      0     % Pitch vel (q)
      0]';   % Yaw vel (r)

% (7.219) fossen
A = [zeros(5,5) eye(5)
     -inv(MRB)*G  -inv(MRB)*D];
B = [zeros(5,5)
     inv(MRB)  ];
 
stop = 100;
x = zeros(stop,10);
x(1,:) = x0;
xdot = zeros(stop,10);

for i = 1:stop
    xdot(i,:) = A*x(i,:)' + B*tau;
    x(i+1,:)  = xdot(i,:) + x(i,:); % Euler integration, now assuming dt = 1, dt*xdot when not true
end

t = 1:i+1;
subplot(2,1,1)
plot(t,x(:,1), t,x(:,2), t,x(:,3));
legend('surge pos', 'sway pos', 'roll pos')

subplot(2,1,2)
plot(t,x(:,6), t,x(:,7), t,x(:,8));
legend('surge vel', 'sway vel', 'roll vel')

%%
% t = 0:0.01:400;
% u = zeros(5,length(t));
% C = eye(10,10);
% sys = ss(A,B,C,0);
% [y,t,x] = lsim(sys,u,t,x0);
% subplot(2,1,1)
% plot(t,x(:,1), t,x(:,2), t,x(:,3));
% legend('surge pos', 'sway pos', 'roll pos')
% 
% subplot(2,1,2)
% plot(t,x(:,6), t,x(:,7), t,x(:,8));
% legend('surge vel', 'sway vel', 'roll vel')
