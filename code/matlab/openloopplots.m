%% Simulation of AAUSHIP in open loop

clear all
r_g = [0.03 0 0.03]'; % location of CG with respect to CO
I_g = [ 6.5406e-02  -1.2600e-02  -5.3593e-02
       -1.2600e-02   1.0892e+00  -1.0800e-03
       -5.3593e-02  -1.0800e-03   1.1068e+00]; % inertia tensor, calculated in inertia.m
m   = 13;             % mass

% rigid body mass matrix, page 53 fossen
MRB = [ m*eye(3)     -m*Smtrx(r_g)
        m*Smtrx(r_g) I_g          ];
MRB = [MRB(1:2,1:2) MRB(1:2,4:6)
       MRB(4:6,1:2) MRB(4:6,4:6)]; % 5DOF, without heave
    

% CRB = m2c(MRB,nu) % rigid body Coriolis matrix, page 56 fossen

% linear hydrodynamic damping
% Xu =  2.86;
% Yv = 32.5;
% Yp = -0.00503
% Yr
% Kv = -0.975;
% Kp =  0.1677;
% Kr
% Mq =  0.0712;
% Nv =  0.975;
% Np = -0.00084;
% Nr

% Data from TP-MB-shipmod.pdf
Xu = -226.2;
Yv = -725.0;
Yp = -3.4;
Yr = 118.2;
Kv = 25.0;
Kp = -3.0;
Kr = 0.8;
Mq = 1;
Nv = -300;
Np = -8.0;
Nr = -290;

D = [Xu  0  0  0  0
      0 Yv Yp  0 Yr
      0 Kv Kp  0 Kr
      0  0  0 Mq  0
      0 Nv Np  0 Nr];
     

% linear restoring forces matrix
G = zeros(5,5);
K_phi   = -0.1385;
M_theta = -0.1788;
G(3,3) = K_phi;
G(4,4) = M_theta;

% Input forces
tau = [ 1 0 0 0 0]';

% Initial states
x(:,1) = [0 0 0 0 0 0 0 0 0 0]';

% (7.219) fossen
A = [zeros(5,5) eye(5)
     -inv(MRB)*G  -inv(MRB)*D] 
B = [zeros(5,5)
     inv(MRB)  ]

for i = 1:10
    xdot(:,i) = A*x(:,i) + B*tau
    x(:,i+1)=xdot(:,i)+x(:,i); % Euler integration, now assuming dt = 1, dt*xdot when not true
end

plot(x(1,:))