clear all
h1 = figure(1); clf;

% Xpos = 1:0.1:10;
% Ypos = 1:0.1:10;
% heading=sin(Xpos/30);
% N = length(Xpos);
N = 399;
Nskip = 20;
%heading=sin(1:0.01:N/100);
heading = ones(1,N)*pi/2;

set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');


mode = 'plot';
mode = 'anim';
if strcmp(mode,'anim')
    Nskip = 1; % Disable skips in animation
end
    hold on
%     plot(Xpos,Ypos,'-')
    hold off

%x = [ u v r x y psi delta ]'
x = zeros(7,N);
xdot = zeros(7,N);
U = zeros(1,N);
error = zeros(1,N);
integral = zeros(1,N);
derivative = zeros(1,N);
Kp = 0.4;
Ki = 10;
Kd = 0;
output = zeros(1,N);

% Main loop
for i = 1:Nskip:N
    [xdot(:,i), U(i)] = mariner(x(:,i), output(i), 5);
    x(:,i+1)=xdot(:,i)+x(:,i);
%     ship(Xpos(i),Ypos(i),heading(i),'r');
    error(i) = heading(i) - x(6,i);
    integral(i) = integral(i) + error(i);
    if i~=1
    derivative(i) = error(i) - error(i-1);
    end
    output(i+1) = Kp*error(i) + Ki*integral(i) + Kd*derivative(i);
        
    figure(1)
%     axis equal
%     xlim([x(4,i)-50 x(4,i)+50])
%     ylim([x(5,i)-50 x(5,i)+50])
    
    ship(x(4,i),x(5,i),rad2pipi(x(6,i)),'r');

end
%     figure(2)
%     subplot(2,1,1)
%     plot(error)
%     subplot(2,1,2)
%     plot(output)





