%% Replay of the simulation data aauship in formation
clear all; clf;
load('simdata.mat')
%%
% set(gcf,'Visible','off'); % Hides the matlab plot because it is ugly
% set(gcf,'paperunits','centimeters')
% set(gcf,'papersize',[13,8]) % Desired outer dimensions of figure
% set(gcf,'paperposition',[-0.5,0,14.5,8.4]) % Place plot on figure





%% Plot the results
t = 0:ts:es*ts-ts;
tt = ts:ts:es*ts;

h = figure(1);
clf

hold on
h1 = plot(track(:,2),track(:,1),'b-o');
shipcolor = [[1 0 0]; [0 1 0]; [0 0 1]; [1.0000    0.8431         0]];

for i = 1:no_boats
    hold on
out = reshape(pij(i,:,1:es), 2, []);
h3 = plot(out(1,1:es),out(2,1:es),'-g');
end
% out = reshape(pir(i,:,1:es), 2, []);
% h4 = plot(out(1,1:es),out(2,1:es),'-b');

% initial
h2 = plot([x(1,1,k),x(1,2,k),x(1,3,k),x(1,4,k),x(1,1,k)],[x(2,1,k),x(2,2,k),x(2,3,k),x(2,4,k),x(2,1,k)],'r--');

% legend([h1;h2;h3;h4],'track','formation','pij','pir')

xlabel('Easting [m]');
ylabel('Northing [m]');
title('Plot of the NED frame');

axis equal
for k = 1:1:k
    tic
    h = findall(gca, 'type', 'patch');
    delete(h)
    h = findall(gca, 'type', 'line', 'color', 'k', 'marker', '+');
    delete(h)
    delete(h2)
    h2 = plot([x(1,1,k),x(1,2,k),x(1,3,k),x(1,4,k)],[x(2,1,k),x(2,2,k),x(2,3,k),x(2,4,k)],'r--');
    for i = 1:no_boats
        hold on
        ship(x(1,i,k),x(2,i,k),x(7,i,k),shipcolor(i,:));


    %     out = reshape(pir(i,:,1:es), 2, []);
    %     plot3(out(1,1:es),out(2,1:es),'-g')
    %     plot3(out(1,1:es),out(2,1:es),Ftotmagn3(1:es,i),'-g')

        hold on
    end
    toc
    

    grid on
    
%     xlim([-250 -120])
%     ylim([-240 -140])
    xdim = max(pij(:,1,k)) - min(pij(:,1,k));
    ydim = max(pij(:,2,k)) - min(pij(:,2,k));
    mxdim = min(pij(:,1,k))+xdim/2;
    mydim = min(pij(:,2,k))+ydim/2;
    dim = max([xdim, ydim])/2+2;
    xlim([mxdim-dim, mxdim+dim])
    ylim([mydim-dim, mydim+dim])

%     axis equal
%     hold off
    
    drawnow('update')
    pause(0.001)
end

text(mxdim,mydim,'The end','HorizontalAlignment','center','FontSize',80,'Interpreter','latex')



