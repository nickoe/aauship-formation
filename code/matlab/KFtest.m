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

%plot(x_hat')

%% Extended Kalman Filter Implementation

clear all;
clf;

load('ssaauship.mat');

PHI = Ad;
G = Bd;

N = 100;

states = 10;
x_hat = zeros(states,N);
P_minus = zeros(states,states,N);
u = [10 0 0 0 0]';
x = zeros(states,N);
x_minus = zeros(states,N);

varians = [0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
R = diag([varians*1.5]);

Q = diag([0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]);

for k = 2:N-1

% Model state vector
x(:,k) = Ad * x(:,k-1) + Bd * u;

%H(:,:,k) = h(k)-h(k-1);
H(:,:,k) = [1 0 0 0 0 0 0 0 0 0;
            0 1 0 0 0 0 0 0 0 0;
            0 0 0 0 1 0 0 0 0 0;
            0 0 0 0 0 1 0 0 0 0;
            0 0 0 0 0 0 1 0 0 0;
            0 0 0 0 0 (x(6,k)-x(6,k-1)) 0 0 0 0;
            0 0 0 0 0 0 (x(7,k)-x(7,k-1)) 0 0 0];

% Add noise, making measurements
z(:,k) = H(:,:,k)*x(:,k) + randn(7,1)*1;
        
% Update
y_tilde(:,k) = z(:,k) - H(:,:,k)*x_minus(:,k);
S(:,:,k) = H(:,:,k)*P_minus(:,:,k)*H(:,:,k)' + R;
K(:,:,k) = P_minus(:,:,k)*H(:,:,k)'*inv(S(:,:,k));
x_hat(:,k) = x_minus(:,k) + K(:,:,k)* y_tilde(:,k);
P_pos(:,:,k) = (eye(10) - K(:,:,k)*H(:,:,k))*P_minus(:,:,k);

% Prediction
x_minus(:,k+1) = PHI*x_hat(:,k) + G*u;
P_minus(:,:,k+1) = PHI*P_minus(:,:,k)*PHI + Q;

%x(:,k+1) = x(:,k) + 0.1*x_hat(:,k);

end

figure(1)
plot(x_hat(1,:),x_hat(2,:))
xlabel('easting [m]'); ylabel('northing [m]')
axis equal;

figure(2)
plot(x_hat(3,:))
title('Phi');

figure(3)
plot(x_hat(4,:))
title('u speed')

figure(4)
plot(x_hat(5,:))
title('v speed')

figure(5)
plot(x_hat(6,:))
title('ax')

figure(6)
plot(x_hat(7,:))
title('ay')

figure(7)
plot(x(1,:),x(2,:))
xlabel('easting [m]'); ylabel('northing [m]');
axis equal;




