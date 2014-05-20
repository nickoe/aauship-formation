function [ xdot ] = aauship( x, tau )
%AAUSHIP Control Model of AAUSHIHP
%   This is a simple model of AAUSHIP, it does not have the fidelity to be
%   called a simulation model, so it is more like a Control Design Model.
%   
%   Since AAUSHIP is a twin propeller ship and has no rudder angle the
%   interface for this is different than the other vessel models used in
%   the MSS toolbox.
%
%   See Property 3.1, chap 7.5+
%   
%   State vector: x = [x y phi theta psi u v p q r]'
%   Force vector: tau = [X Y K M N]'

% Check of input and state dimensions
if (length(x)  ~= 10),error('x-vector must have dimension 10!');end
if (length(tau) ~= 5),error('tau-vector must have dimension 5!');end

ss = load('ssaauship.mat');
xdot = ss.Ad*x + ss.Bd*tau;
end

