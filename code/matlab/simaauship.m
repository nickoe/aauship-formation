clear all; clf;

N = 300;
x = zeros(N,10);
x(1,:) = [0 0 0 0 0 3.5 0 0 0 0]';
xdot = zeros(N,10);
tau = [1 0 0 0 0.01]';
NED = zeros(N,2);
for k = 1:N
    x(k+1,:) = aauship(x(k,:)', tau);
    psi=x(k,5);
    Rz = [cos(psi) -sin(psi);
          sin(psi)  cos(psi)];
    if k ~=  1
    NED(k,:) = Rz*x(k,6:7)' + NED(k-1,:)';
    end
end

figure(1)
% plot(x(:,2),x(:,1),'.-')
plot(NED(:,1),NED(:,2))