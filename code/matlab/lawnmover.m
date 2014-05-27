%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%lawn mower generator%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
clear all
% Box options
boxheigth = 500;
boxwidth = 700;

ll = [0,0];
lr = [boxwidth,0];
ur = [boxwidth,boxheigth];
ul = [0,boxheigth];


% Turning restrictions
tr = 40; % turning radius
n = 1;
figure(1)
clf
hold on
while ((lr(2) + n*tr) <= (ur(2) - tr))
rightwps(n,:) = [lr(1)-tr , lr(2)+n*tr]
leftwps(n,:) = [tr,rightwps(n,2)]
plot([leftwps(n,1),rightwps(n,1)],[leftwps(n,2),rightwps(n,2)])
n = n+1
end
N=n-1;
k=1
for n = 1:N
if mod(n,2)==1 %ulige
    allwps(k,:) = leftwps(n,:)
    k = k+1
    allwps(k,:) = rightwps(n,:)
    k = k+1
    n = n+1
elseif mod(n,2)==0 %lige
    allwps(k,:) = rightwps(n,:)
    k = k+1
    allwps(k,:) = leftwps(n,:)
    k = k+1
    n = n+1
end
end

plot(allwps(:,1),allwps(:,2))




