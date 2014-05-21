%% Simulation of aauship

clear all; clf;

N = 1000;
x = zeros(N,10);
x(1,:) = [0 0 0 0 0 2 0 0 0 0]';
xdot = zeros(N,10);
tau = [10 0 0 0 -0.05]';
NED = zeros(N,2);

for k = 1:N
    x(k+1,:) = aauship(x(k,:)', tau);
    psi=x(k,5);
    Rz = [cos(psi) -sin(psi);
          sin(psi)  cos(psi)];
    if k ~=  1
    NED(k,:) = Rz*x(k,6:7)'*0.1 + NED(k-1,:)';
    end
end

%% Plot the results
figure(1)
for k = 1:10:N
    ship(NED(k,2),NED(k,1),-x(k,5)+pi/2,'y')
end
hold on
plot(NED(:,2),NED(:,1))
xlabel('Easting [m]');
ylabel('Northing [m]');
grid on
axis equal
hold off