%% Simulation of aauship
% TODO nonlinear stuff for simulation model

clear all; clf;

N = 6000;
x = zeros(N,10);
x(1,:) = [0 0 0 0 0 2 0 0 0 0]';
xdot = zeros(N,10);
taus = [1 0 0 0 0.005]';
tau = repmat(taus',N,1);
% tau(:,1) = (1:N)/600;
taus = [12 0 0 0 0]';
tau(ceil(N/2)+1:N,:)  = repmat(taus',N/2,1);
%% 
NED = zeros(N,2);
lx1 = 0.41; lx2 = 0.18; lx3 = 0.48; lx4 = 0.48; ly3 = 0.05; ly4 = 0.05;
lz3 = 0.05; lz4 = 0.05;
a = atan(ly3/lx3);
az = atan(lz3/lz3);
T = [ 0 0 1 1;...
      1 1 -sin(a) sin(a);...
      -1 -1 0 0;...
      0 0 sin(az)*lz3 sin(az*lz4);...
      lx1 -lx2 -sin(a)*lx3 sin(a)*lx4];
K = eye(4,4);
K(3,3) = 0.0005;
K(4,4) = 0.0005;
u = [0 0 150 200]';
tau = T*K*u;

for k = 1:N

    x(k+1,:) = aauship(x(k,:)', tau);
%     x(k+1,:) = aauship(x(k,:)', tau(k,:)');
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
% for k = 1:10:N
%     ship(NED(k,2),NED(k,1),-x(k,5)+pi/2,'y')
% end
hold on
plot(NED(:,2),NED(:,1))
xlabel('Easting [m]');
ylabel('Northing [m]');
grid on
axis equal
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

