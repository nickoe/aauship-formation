function [ xn, eta, nu, nudot ] = aaushipsimmodel( x, tau )
%AAUSHIP Control Model of AAUSHIHP
%   This is a simple model of AAUSHIP, supposed to be a simulation model
%   
%   Since AAUSHIP is a twin propeller ship and has no rudder angle the
%   interface for this is different than the other ship models used in
%   the MSS toolbox.
% 
%   State vector: x = [x y phi theta psi u v p q r]'
%   Position vector: eta = [N E phi theta psi]'
%   Velocity vector: nu = [u v p q r]'
%   Force input vector: tau = [X Y K M N]'

% Check of input and state dimensions
if (length(x)  ~= 10),error('x-vector must have dimension 10!');end
if (length(tau) ~= 5),error('tau-vector must have dimension 5!');end

% TODO Add control allocation matrix and saturation with printable warnings

% Linear simulation step
ss = load('ssaauship.mat');
xn = ss.Ad*x + ss.Bd*tau;

% Calculate northing and easting position with euler integration
xn(5)      = xn(10)*ss.ts + x(5);
Rz         = [cos(xn(5)) -sin(xn(5)); sin(xn(5)) cos(xn(5))];
R          = diag(ones(5,1));
R(1:2,1:2) = Rz(1:2,1:2);
eta(1:5)   = x(1:5) + ss.ts*R*xn(6:10);
xn(1:5)    = eta(1:5);

% Compute fossen vectors
eta(3:5)   = xn(8:10)*ss.ts+x(3:5);        % Positions
nu         = xn(6:10);                  % Velocities
nudot      = diff([x(6:10) xn(6:10)]'); % Accelerations
end
