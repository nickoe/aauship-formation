function [ xdot ] = aauship( x, tau )
%AAUSHIP Control Model of AAUSHIHP
%   This is a simple model of AAUSHIP, it does not have the fidelity to be
%   called a simulation model, so it is more like a Control Design Model.
%   
%   Since AAUSHIP is a twin propeller ship and has no rudder angle the
%   interface for this is different than the other vessel models used in
%   the MSS toolbox.
%   
%   See the MSS toolbox supply.m for implementation example. The matrices M
%   and D has to be determined.
%
%   See Property 3.1, chap 7.5+
%   
%   State vector: x = [u v r x y theta]

% Check of input and state dimensions
if (length(x)  ~= 6),error('x-vector must have dimension 6!');end
if (length(tau) ~= 2),error('tau-vector must have dimension 2!');end

theta = 10*pi/180; %[rad]
m = 12; %[kg]

% Friction coefficients
kx = -0.01;
ky = -0.1;
kz = -0.1;

% WARNING thist is only including the one propeller!!!! we need to remmeber
% the other else we will just be circling or something like that
xdot(1) = kx*abs(x(1))*x(1) + (cos(theta)^2)*tau(1)/m;
xdot(2) = ky*abs(x(2))*x(2) + cos(theta)*sin(theta)*tau(1)/m;
xdot(3) = kz*abs(x(3))*x(3) + sin(theta)*tau(1)/m;
xdot(4) = xdot(1) + x(1);
xdot(5) = xdot(2) + x(2);
xdot(6) = xdot(3) + x(3);
end

