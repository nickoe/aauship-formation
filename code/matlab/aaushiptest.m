% test for aauship.m
clear all
N = 40;
clf


%x = [ u v r x y psi ]'
x = zeros(6,N);
xdot = zeros(6,N);

for i = 1:N
    [xdot(1:6,i)] = aauship(x(1:6,i), [5;10]); % SHIP
    x(:,i+1)=xdot(:,i)+x(:,i); % Euler integration, now assuming dt = 1, dt*xdot when not true
end

subplot(2,1,1)
plot(x(5,:),x(4,:),'-r.');
ship(x(5,1:N),x(4,1:N),rad2pipi(-1*x(6,1:N)+pi/2),'r');
ylabel('Northing [m]')
xlabel('Easting [m]')
axis equal
subplot(2,1,2)
plot(x(6,:),'.-')
xlabel('Time [samples]')
ylabel('Heading angle [rad]')
