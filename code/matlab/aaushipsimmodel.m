function [ xn, eta, nu, nudot ] = aaushipsimmodel( x, tau, ts )
%AAUSHIP Control Model of AAUSHIHP
%   This is a simple model of AAUSHIP, supposed to be a simulation model
%   
%   Since AAUSHIP is a twin propeller ship and has no rudder angle the
%   interface for this is different than the other vessel models used in
%   the MSS toolbox.
%
%   See Property 3.1, chap 7.5+
%   
%   State vector: x = [x y phi theta psi u v p q r]'
%   Position vector: eta = [N E phi theta psi]'
%   Velocity vector: nu = [u v p q r]'
%   Force input vector: tau = [X Y K M N]'

% Check of input and state dimensions
if (length(x)  ~= 10),error('x-vector must have dimension 10!');end
if (length(tau) ~= 5),error('tau-vector must have dimension 5!');end

ss = load('ssaauship.mat');

xn = ss.Ad*x + ss.Bd*tau;

xn(5) = xn(10)*ts + x(5);
psi=xn(5);
Rz = [cos(psi) -sin(psi);
      sin(psi)  cos(psi)];
R = diag(ones(5,1));
R(1:2,1:2) = Rz(1:2,1:2);
eta(1:5) = x(1:5) + ts*R*xn(6:10);
xn(1:5) = eta(1:5);

eta(3:5) = xn(8:10)*ts+x(3:5);
nu = xn(6:10);

nudot=diff([x(6:10) xn(6:10)]');
end
