%% Linear Kalman Filter Implementation

clear all;

load('ssaauship.mat');
PHI = Ad;
DELTA = Bd;

N = 100;
states = 10;
Q = eye(states)*0.01;
varians = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
R = diag([varians*1.5]);
x_bar = zeros(states,N);
x_hat = zeros(states,N);
P_bar = zeros(states,states,N);
H = diag([0 0 0 0 0 0 0 1 1 1]);
u = zeros(5,N);
x = zeros(states,N);

for k = 1:N-1
%     y(:,k) = x_bar(:,k) + randn(10,1)*0.1;
    x_dot(:,k) = Ad*x(:,k) + Bd*u(:,k);
    
    y(:,k) = x_dot(:,k)+randn(10,1 );
    K(:,:,k) = P_bar(:,:,k)*H'* inv((H*P_bar(:,:,k)*H' + R));
    x_hat(:,k) = x_bar(:,k) + K(:,:,k)* (y(:,k) - H*x_hat(:,k));
    P_hat(:,:,k) = (eye(states) - K(:,:,k)*H)*P_bar(:,:,k);
    
    x_bar(:,k+1) = PHI*x_hat(:,k) + DELTA*u(:,k);
    P_bar(:,:,k+1) = PHI*P_hat(:,:,k)*PHI' + Q;
    
    x(:,k+1) = x(:,k) + 0.1*x_dot(:,k);
end    

plot(x_hat')

%% Extended Kalman Filter Implementation

clear all;



