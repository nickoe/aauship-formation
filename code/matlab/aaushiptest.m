% test for aauship.m
clear all
N = 30;


%x = [ u v r x y psi ]'
x = zeros(6,N);
xdot = zeros(6,N);

for i = 1:N
    [xdot(1:6,i)] = aauship(x(1:6,i), [70;0]); % SHIP
    x(:,i+1)=xdot(:,i)+x(:,i); % Euler integration, now assuming dt = 1, dt*xdot when not true
end

subplot(2,1,1)
plot(x(5,:),x(4,:),'r.');
ylabel('Northing [m]')
xlabel('Easting [m]')
subplot(2,1,2)
plot(xdot(6,:))
xlabel('Time [samples]')
ylabel('Heading angle [rad]')
axis equal