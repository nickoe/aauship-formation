%% Linear Kalman Filter Implementation
% 
% clear all;
% 
% load('ssaauship.mat');
% PHI = Ad;
% DELTA = Bd;
% 
% N = 100;
% states = 10;
% Q = eye(states)*0.01;
% varians = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
% R = diag([varians*1.5]);
% x_bar = zeros(states,N);
% x_hat = zeros(states,N);
% P_bar = zeros(states,states,N);
% H = diag([0 0 0 0 0 0 0 1 1 1]);
% u = zeros(5,N);
% x = zeros(states,N);
% 
% for k = 1:N-1
% %     y(:,k) = x_bar(:,k) + randn(10,1)*0.1;
%     x_dot(:,k) = Ad*x(:,k) + Bd*u(:,k);
%     
%     y(:,k) = x_dot(:,k)+randn(10,1 );
%     K(:,:,k) = P_bar(:,:,k)*H'* inv((H*P_bar(:,:,k)*H' + R));
%     x_hat(:,k) = x_bar(:,k) + K(:,:,k)* (y(:,k) - H*x_hat(:,k));
%     P_hat(:,:,k) = (eye(states) - K(:,:,k)*H)*P_bar(:,:,k);
%     
%     x_bar(:,k+1) = PHI*x_hat(:,k) + DELTA*u(:,k);
%     P_bar(:,:,k+1) = PHI*P_hat(:,:,k)*PHI' + Q;
%     
%     x(:,k+1) = x(:,k) + 0.1*x_dot(:,k);
% end    
% 
% %plot(x_hat')

%% Extended Kalman Filter Implementation

clear all;
clf;

load('ssaauship.mat');

PHI = Ad;
G = Bd;

N = 200;

states = 10;
x_hat_plus = zeros(states,N);
P_minus = zeros(states,states,N);
u = [5 0 0 0 -0.1]';
x = zeros(states,N);
x_hat_minus = zeros(states,N);

% Process noise
w = [0.5 0.5 3.4182e-006 3.1662e-006 13.5969e-006 0.1 0.1 332^-6 332^-6 332^-6]';
% w = zeros(10,1);

% Measurement noise
v = [1.5 1.5 13.5969e-006 0.1 0.1 0.0524 0.0524]';
% v = zeros(7,1);

% jeppe = [1 1 1 1 1 1 1]';
% R = diag(jeppe*500);
R = diag(v);
Q = diag(w);

NED = zeros(2,N);
NED_noisy = zeros(2,N);
heading(1) = x(1,5);

for k = 2:N

% Model state vector
% x(:,k) = Ad * x(:,k-1) + Bd * u;
x(:,k) = aauship(x(:,k-1), u);
noise(:,k) = randn(10,1).*w;
x_noisy(:,k) = x(:,k) + noise(:,k);

%H(:,:,k) = h(k)-h(k-1);
H(:,:,k) = [1 0 0 0 0 0 0 0 0 0;
            0 1 0 0 0 0 0 0 0 0;
            0 0 0 0 1 0 0 0 0 0;
            0 0 0 0 0 1 0 0 0 0;
            0 0 0 0 0 0 1 0 0 0;
            0 0 0 0 0 (x_noisy(6,k)-x_noisy(6,k-1)) 0 0 0 0;
            0 0 0 0 0 0 (x_noisy(7,k)-x_noisy(7,k-1)) 0 0 0];

% Add noise, making measurements
z(:,k) = H(:,:,k)*x_noisy(:,k) + randn(7,1).*v;
        
% Update
z_hat(:,k) = z(:,k) - H(:,:,k)*x_hat_minus(:,k);
S(:,:,k) = H(:,:,k)*P_minus(:,:,k)*H(:,:,k)' + R;
K(:,:,k) = P_minus(:,:,k)*H(:,:,k)'*inv(S(:,:,k));
x_hat_plus(:,k) = x_hat_minus(:,k) + K(:,:,k)* z_hat(:,k);
% P_plus(:,:,k) = (eye(10) - K(:,:,k)*H(:,:,k))*P_minus(:,:,k);
P_plus(:,:,k) = (eye(10) - K(:,:,k)*H(:,:,k)) *P_minus(:,:,k)* (eye(10) - K(:,:,k)*H(:,:,k))' + K(:,:,k)*R*K(:,:,k)';

% Prediction
% x_hat_minus(:,k) = PHI*x_hat_plus(:,k-1) + G*u;
% P_minus(:,:,k) = PHI*P_minus(:,:,k-1)*PHI + Q;
x_hat_minus(:,k+1) = PHI*x_hat_plus(:,k) + G*u;
P_minus(:,:,k+1) = PHI*P_plus(:,:,k)*PHI + Q;

%x(:,k+1) = x(:,k) + 0.1*x_hat(:,k);




psi=x(5,k);
Rz = [cos(psi) -sin(psi);
      sin(psi)  cos(psi)];

      
NED(:,k+1) = Rz*x(6:7,k)*0.1 + NED(:,k);
psi=x_hat_plus(5,k);
Rz = [cos(psi) -sin(psi);
      sin(psi)  cos(psi)];
NED_noisy(:,k+1) = Rz*x_hat_plus(6:7,k)*0.1 + NED_noisy(:,k);
% heading(k) = (x(10,k)'*0.1 + heading(k-1));
end


figure(1)
% plot(x_hat_plus(1,:),x_hat_plus(2,:),'.-', x(1,:),x(2,:),'.-')
plot(NED(1,:),NED(2,:),NED_noisy(1,:),NED_noisy(2,:))
xlabel('easting [m]'); ylabel('northing [m]')
legend('x', 'x_{hat}')
axis equal;

% for k = 1:N
%     k
%     ship(x_hat(1,:),x_hat(2,:),-x_hat(5,:)+pi/2,'y')
% end

figure(2)
plot(1:N,x_noisy(5,:), 1:N,x_hat_plus(5,:), 1:N,x(5,:) )
legend('Psi_{noisy}', 'Psi_{hat}', 'Psi');

figure(5)
plot(1:N,x_noisy(6,:), 1:N,x_noisy(7,:), 1:N,x_hat_plus(6,:), 1:N,x_hat_plus(7,:), 1:N,x(6,:), 1:N,x(7,:))
legend('u_{noisy}', 'u_{noisy}', 'u_{hat}', 'v_{hat}','u', 'v')





