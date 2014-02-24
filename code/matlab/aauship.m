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

end

