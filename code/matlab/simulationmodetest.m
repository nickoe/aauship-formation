% This tests the simulaiton model and plots stuff
clear all;


% Input vector
tau = [6 0 0 0 -0.1]';

% Sample period
ts = 0.1;

% Number of samples
N = 500;

x = zeros(10,N);
eta = zeros(5,N);
nu = zeros(5,N);
nudot = zeros(5,N);



% Initial state
x(:,1) = [0 0 0 0 0.7854 0 0 0 0 0]';

for k = 1:N-1
    if k > 200
        tau = [ 1 0 0 0 0.1 ]';
    end
    [x(:,k+1), eta(:,k+1), nu(:,k+1), nudot(:,k+1)] = aaushipsimmodel( x(:,k), tau, ts );

    z(1:2,k+1) = eta(1:2,k+1);
    z(3,k+1) = eta(5,k+1);
    z(4:5,k+1) = nu(1:2,k+1);
    z(6:7,k+1) = nudot(1:2,k+1);
end

figure(1)
subplot(2,2,1)
plot(eta(2,:),eta(1,:),'-')
axis equal
xlabel('Easting (m)')
ylabel('Norting (m)')

subplot(2,2,2);
plot(ts:ts:N*ts,eta(5,:))
xlabel('Time (s)')
ylabel('Heading (rad)')

subplot(2,2,3);
plot(ts:ts:N*ts,nu(1,:), ts:ts:N*ts,nu(2,:))
xlabel('Time (s)')
ylabel('Speed (m/s)')

subplot(2,2,4);
plot(ts:ts:N*ts,z(6,:), 'b', ts:ts:N*ts,z(7,:), 'g')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')