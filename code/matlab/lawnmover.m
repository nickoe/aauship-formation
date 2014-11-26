%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%lawn mower generator%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Updated smaller track
clear all
% Box options
boxheigth = 45;
boxwidth = 30;

% ll = [0,0];
lr = [boxwidth,0];
ur = [boxwidth,boxheigth];
% ul = [0,boxheigth];

load('/afs/ies.auc.dk/group/14gr1034/Private/matlab/gpx2local/klingenberg.mat')

% Turning restrictions
tr = 4; % turning radius
s = 2*tr;
n = 1;
figure(1)
clf
hold on
while ((lr(2) + n*s) <= (ur(2) - s))
rightwps(n,:) = [lr(1)-s , lr(2)+n*s]
leftwps(n,:) = [s,rightwps(n,2)]
plot([leftwps(n,1),rightwps(n,1)],[leftwps(n,2),rightwps(n,2)])
n = n+1
end

% y = [-tr:0.1:tr]';
y = [-tr+0.0001:0.01:tr-0.001]';
x = sqrt((tr)^2-y.^2);
cl = length(x)-1;

g = 1;
k = 1;
for l = 2:cl
%     norm([x(l)-x(l+1),y(l)-y(l+k)])
    if ( norm([x(l-k)-x(l),y(l-k)-y(l)]) > 1.5)
        xx(g) = x(l);
        yy(g) = y(l);
        g = g  +1;
        k = 1;
    else
        k = k+1;
    end
    l = l+1;
end
clear x y
x = xx';
y = yy';
cl = length(y)-1;

N=n-1;
k=1;
hold on
for n = 1:N
if mod(n,2)==1 %ulige
    hold on
    allwps(k,:) = leftwps(n,:);
    k = k+1
    allwps(k,:) = rightwps(n,:);
    k = k+1
    plot(allwps(k-1,1),tr+allwps(k-1,2),'*')
    allwps(k:k+cl,:) = [x+allwps(k-1,1),y+tr+allwps(k-1,2)];
%     allwps(k:k+cl,:) = [allwps(k-1,1)+sqrt((tr)^2-y.^2+2.*y.*allwps(k-1,2)-allwps(k-1,2).^2),y];
    k = k+cl;
    n = n+1
elseif mod(n,2)==0 %lige
    allwps(k,:) = rightwps(n,:);
    k = k+1
    allwps(k,:) = leftwps(n,:);
    k = k+1
    plot(allwps(k-1,1),tr+allwps(k-1,2),'*')
    allwps(k:k+cl,:) = [-(x-allwps(k-1,1)),y+tr+allwps(k-1,2)];
    k = k+cl;
    n = n+1
end
end
% save('track.mat','track')

figure(1)
plot(allwps(:,1),allwps(:,2),'.-')
axis equal

% allwps(:,2) = allwps(:,2)+(-57);
% allwps(:,1) = allwps(:,1)+(-46.5);
allwps(:,2) = allwps(:,2)+(-57);
allwps(:,1) = allwps(:,1)+(-34);

allwps(1,2) = -49;
allwps(1,1) = -20;

figure(2)
plot(inner(:,2),inner(:,1),'b', outer(:,2),outer(:,1),'g', allwps(:,2),allwps(:,1),'.-r')
axis equal
track = [allwps(:,1) allwps(:,2)];
save('lawnmoversmall.mat','track')


%% Just a simple triangle wp
clear all;
load('/afs/ies.auc.dk/group/14gr1034/Private/matlab/gpx2local/klingenberg.mat')

allwps(:,1) = [-30 -13 -15];
allwps(:,2) = [-35 -30 -50];

figure(3)
plot(inner(:,2),inner(:,1),'b', outer(:,2),outer(:,1),'g',allwps(:,2),allwps(:,1),'.-r')
axis equal

track = [allwps(:,1) allwps(:,2)];
save('triangletrack.mat','track')

%% Only a small line segment
clear all;
load('/afs/ies.auc.dk/group/14gr1034/Private/matlab/gpx2local/klingenberg.mat')

allwps(:,1) = [-24 -18];
allwps(:,2) = [-40 -30];

figure(4)
plot(inner(:,2),inner(:,1),'b', outer(:,2),outer(:,1),'g',allwps(:,2),allwps(:,1),'.-r')
axis equal

track = [allwps(:,1) allwps(:,2)];
save('linesegment.mat','track')

%% Very small line segment
clear all;
load('/afs/ies.auc.dk/group/14gr1034/Private/matlab/gpx2local/klingenberg.mat')

allwps(:,1) = [-12 -11];
allwps(:,2) = [-20 -19];

figure(5)
plot(inner(:,2),inner(:,1),'b', outer(:,2),outer(:,1),'g',allwps(:,2),allwps(:,1),'.-r')
axis equal

track = [allwps(:,1) allwps(:,2)];
save('verysmalllinesegment.mat','track')

%% Larger line segment
clear all;
load('/afs/ies.auc.dk/group/14gr1034/Private/matlab/gpx2local/klingenberg.mat')

allwps(:,1) = [-30 -11];
allwps(:,2) = [-50 -19];

figure(5)
plot(inner(:,2),inner(:,1),'b', outer(:,2),outer(:,1),'g',allwps(:,2),allwps(:,1),'.-r')
axis equal

track = [allwps(:,1) allwps(:,2)];
save('largelinesegment.mat','track')







