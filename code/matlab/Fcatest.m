clear all;

%% 3D plot with force magnitude
MPI = pi;
% Laver grid med meshgrid, step bestemmer 'opløsning'
step = 0.2;
[X,Y] = meshgrid(-100:step:100,-100:step:100);
% Vi transponerer her da det ellers ikke passer, af en eller anden årsag
X=X';
Y=Y';
lenx=length(X(:,1));
leny=length(Y(1,:));
% Safe avoidance radius
rsav = 30;
% Placering af virtuel leader, pt underordnet
vl = [10,10];
% Pos af hvor baad i skal ende (Midten af plot)
pi0 = [7,3];
% Gains til funktionerne
Kvl = 0.3;
Kij = 0.1;
Kca = 240;
Koa = 120;
% Init af felterne
Ftot = zeros(lenx, leny,2);
Ftotmagn = zeros(lenx,leny);
Fvlmagn = zeros(lenx,leny);
Fijmagn = zeros(lenx,leny);
Fcamagn = zeros(lenx,leny);
Foamagn = zeros(lenx,leny);

% Placering (start og slut) af andre både udover båd i
pj(1,1:2) = [90 , 50];
pj(2,1:2) = [90 , -90];
pj(3,1:2) = [-80,80];
pj0(1,1:2) = [-35 , -1];
pj0(2,1:2) = [100 , -100];
pj0(3,1:2) = [-10,-10];

% Placering af forhindinger
po(1,1:2) = [-40,-40];
po(2,1:2) = [-60,-60];
po(3,1:2) = [-30,10];

Fmax = 80;

tic
for m = 1:lenx;
    for n = 1:leny;
        pi = [X(m,1),Y(1,n)];
        
        [Fvlmagn(m,n), Fijmagn(m,n), Fcamagn(m,n), Foamagn(m,n)] = potfield(pi, pi0, pj, pj0, po, vl, Fmax, Kvl, Kij, Kca, Koa, rsav);

        Ftotmagn(m,n) = Fvlmagn(m,n)+Fijmagn(m,n)+Fcamagn(m,n)+Foamagn(m,n);
        Ftotmagn(m,n) = min([norm(Ftotmagn(m,n)),Fmax])*Ftotmagn(m,n)/norm(Ftotmagn(m,n));
        
        % For at danne et 'låg' på plots hvor de ellers ville være NaN da
        % de er meget større end Fmax
        if isnan(Ftotmagn(m,n))
            Ftotmagn(m,n)  = Fmax;
        end
    end
end
toc
%%
% figure(1)
% clf;
% hold on
% axis equal
% density = 10;
% [xvel,yvel] = gradient(-Fvlmagn(1:density:m,1:density:n),step,step);
% % contour(X, Y, Fvlmagn);
% % quiver(X(1:density:m,1:density:n), Y(1:density:m,1:density:n),xvel,yvel);
% title('Contour and quiver plot of Fvl')
% hold off
% figure(2)
% clf;
% hold on
% surf(X, Y, Fvlmagn);
% % contour(X, Y, Fvlmagn);
% title('Fvlmagn')
% hold off
% figure(3)
% clf;
% hold on
% surf(X,Y, Fijmagn);
% axis equal
% title('Fijmagn')
% hold off
% figure(4)
% clf;
% surf(X,Y,Fcamagn)
% title('Fcamagn')
% figure(5)
% clf;
% surf(X,Y,Foamagn)
% title('Foamagn')
% figure(6)
% clf;
% surf(X,Y,Ftotmagn);
% title('Ftotmagn')
figure(7)
clf;
hold on
% density = 4;
% [totxvel,totyvel] = gradient(-Ftotmagn(1:density:m,1:density:n),step,step);
% contour(X, Y, Ftotmagn);
% quiver(X(1:density:m,1:density:n), Y(1:density:m,1:density:n),totxvel,totyvel);
clear m, clear n
% m(1) = 40;
% n(1) = 40;
% for k = 1:1000;
%     A = Ftotmagn(m(k)-1:m(k)+1,n(k)-1:n(k)+1);
%     [minval,index] = min(A(:));
%     [i,j] = ind2sub(size(A),index);
%     m(k+1) = m(k)+(i-2);
%     n(k+1) = n(k)+(j-2);
%     xny(k) = X(m(k),n(k));
%     yny(k) = Y(m(k),n(k));
%     Ftotmagnny(k) = Ftotmagn(m(k),n(k));
% end
% plot3(xny,yny,Ftotmagnny,'r-*')
% plot3(X(m(1),n(1)),Y(m(1),n(1)),Ftotmagn(m(1),n(1)),'bo')
surf(X,Y,Ftotmagn,'EdgeColor','none');
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
% hold off

% New path planner thing
n = 600;
pip = zeros(n,2);
pip(1,1:2) = [-90,-90];
Ftotmagn3 = zeros(n,1);
for k = 1:n
    [pip(k+1,:) , minval] = pathgen(60, 1, pip(k,:), pi0, pj, pj0, po, vl, Fmax, Kvl, Kij, Kca, Koa, rsav);
    Ftotmagn3(k+1) = minval;
end

plot3(pip(:,1),pip(:,2),Ftotmagn3+2,'r-*')