%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%lawn mower generator%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
clear all
% Box options
boxheigth = 600;
boxwidth = 600;

ll = [0,0];
lr = [boxwidth,0];
ur = [boxwidth,boxheigth];
ul = [0,boxheigth];


% Turning restrictions
tr = 10; % turning radius
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

y = [-tr:1:tr]';
x = sqrt((tr)^2-y.^2+tr/2);
cl = length(x)-1;

N=n-1;
k=1
for n = 1:N
if mod(n,2)==1 %ulige
    allwps(k,:) = leftwps(n,:);
    k = k+1
    allwps(k,:) = rightwps(n,:);
    k = k+1
    allwps(k:k+cl,:) = [+(x+allwps(k-1,1)),y+tr+allwps(k-1,2)];
    k = k+cl;
    n = n+1
elseif mod(n,2)==0 %lige
    allwps(k,:) = rightwps(n,:);
    k = k+1
    allwps(k,:) = leftwps(n,:);
    k = k+1
    allwps(k:k+cl,:) = [-(x-allwps(k-1,1)),y+tr+allwps(k-1,2)];
    k = k+cl;
    n = n+1
end
end

save('track.mat','allwps')
plot(allwps(:,1),allwps(:,2),'-')
axis equal



