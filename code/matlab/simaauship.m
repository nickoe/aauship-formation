%% Simulation of aauship
% TODO nonlinear stuff for simulation model

clear all; clf;

N = 6000;
x = zeros(N,10);
x(1,:) = [0 0 0 0 0 0 0 0 0 0]';
xdot = zeros(N,10);
taus = [1 0 0 0 0.005]';
tau = repmat(taus',N,1);
% tau(:,1) = (1:N)/600;
taus = [10 0 0 0 0]';
tau(ceil(N/2)+1:N,:)  = repmat(taus',N/2,1);
%% 
NED = zeros(N,2);

for k = 1:N
    x(k+1,:) = aauship(x(k,:)', tau(k,:)');
    psi=x(k,5);
    Rz = [cos(psi) -sin(psi);
          sin(psi)  cos(psi)];
    if k ~=  1
    NED(k,:) = Rz*x(k,6:7)'*0.1 + NED(k-1,:)';
    end
end

%% Plot the results
figure(1)
clf
for k = 1:10:N
    ship(NED(k,2),NED(k,1),-x(k,5)+pi/2,'y')
end
hold on
plot(NED(:,2),NED(:,1))
xlabel('Easting [m]');
ylabel('Northing [m]');
grid on
% axis equal
hold off

figure(2)
subplot(3,1,1)
plot(0:N,x(:,6))
ylabel('Surge speed [m/s]')
subplot(3,1,2)
plot(0:N,x(:,7))
ylabel('Sway speed [m/s]')
subplot(3,1,3)
plot(0:N,x(:,10))
ylabel('Yaw speed [rad/s]')
xlabel('Time [s]')

figure(3)
subplot(3,1,1)
plot(0:N,x(:,3))
ylabel('Rool angle [rad]')
subplot(3,1,2)
plot(0:N,x(:,4))
ylabel('Pitch angle [rad]')
subplot(3,1,3)
plot(0:N,x(:,5))
ylabel('Yaw angle [rad]')
xlabel('Time [s]')

