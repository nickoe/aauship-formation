%% This tests the simulaiton model and plots stuff
clear all;


% Input vector
tau = [6 0 0 0 -0.1]';

% Sample period
ts = 0.1;

% Number of samples
N = 1000;

x = zeros(17,N);% Full state simulation vector
z = zeros(10,N);
eta = zeros(5,N);
c = zeros(2,N-1);

nu = zeros(5,N);
nudot = zeros(5,N);

% Initial state
x(:,1) = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';

% ROS node setup
node = rosmatlab.node('matlab_sim_node', 'http://localhost:11311');
pub = node.addPublisher('simstate', 'std_msgs/Float64MultiArray');
msg = rosmatlab.message('std_msgs/Float64MultiArray', node);

flag=1;
%% Simulation
for k = 1:N-1
    % Zig-zag
    if z(3,k) > deg2rad(30+90) && flag==0
        tau(5) = -tau(5);
        flag=1;
    elseif z(3,k) < deg2rad(-30+90) && flag==1
        tau(5) = -tau(5);
        flag=0;
    end
    
    % Simulation model
    [x(:,k+1), eta(:,k+1), nu(:,k+1), nudot(:,k+1)] = aaushipsimmodel( x(:,k), tau);
    
    Rz         = [cos(x(7)) -sin(x(7)); sin(x(7)) cos(x(7))];
    c(:,k) = Rz*x(3:4,k);
    % Measurement vector
    z(1:2,k+1) = eta(1:2,k+1);
    z(3,k+1) = eta(5,k+1);
    z(4:5,k+1) = nu(1:2,k+1);
    z(6:7,k+1) = nudot(1:2,k+1);
    
    % ROS state publish
    pause(1)
    msg.setData(x(:,k+1));
    pub.publish(msg);
end

%% Plotting
figure(1)
subplot(2,2,1)
% plot(z(2,:),z(1,:),'-')
plot(x(2,:),x(1,:),'-')
axis equal
xlabel('Easting (m)')
ylabel('Norting (m)')

subplot(2,2,2);
plot(ts:ts:N*ts,z(3,:))
xlabel('Time (s)')
ylabel('Heading (rad)')

subplot(2,2,3);
plot(ts:ts:N*ts,z(4,:), ts:ts:N*ts,z(5,:))
xlabel('Time (s)')
ylabel('Speed (m/s)')

subplot(2,2,4);
plot(ts:ts:N*ts,z(6,:), 'b', ts:ts:N*ts,z(7,:), 'g')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')