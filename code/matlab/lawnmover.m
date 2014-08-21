%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%lawn mower generator%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
clear all
% Box options
boxheigth = 142;
boxwidth = 202;

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

save('track.mat','allwps')
plot(allwps(:,1),allwps(:,2),'.-')
axis equal



