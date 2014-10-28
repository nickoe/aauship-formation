close all; clear all;


%% Plot echelon
clear all;
xpos = [0, 1, 2, 3, 4, 5];
ypos = [0, -1, -2, -3, -4, -5];
heading = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2];

figure(1)
hold on
%plot(xpos,ypos,'*') %Lige for at teste
ship(xpos,ypos,heading,'y')
xlabel('Easting [m]');
ylabel('Northing [m]');
grid on
axis equal
hold off


%% Plot potential field w limited communication
clear all; close all;

x = linspace(0,pi);
y = linspace(0,pi);
[X,Y] = meshgrid(x,y);

Z = sin(-X) + sin(-Y) + 5;

z_min = min(Z(:));

%plane = surf(X,Y,5)

figure(2)
hold on
skip1 = surf(X,Y,Z) % centrum i [pi/2,pi/2]
skip2 = surf(X+pi,Y+pi,Z) % centrum i [3pi/2,3pi/2]
skip3 = surf(X+2*pi,Y,Z) % centrum i [5pi/2,pi/2]
grid on
axis equal
hold off


%% Plot potential field w full communication















