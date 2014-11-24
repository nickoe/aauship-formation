clear all; 

% %% Fca
% rsav = 3;
% Kca = 30;
% d = 0.01:0.1:10;
% N = length(d);
% 
% % exmple pf eg. (4) from helicopter potential field paper
% for k=1:N
%     if d(k) < rsav
%         F(k) = ( (Kca*rsav)/abs(d(k))-Kca ) * d(k)/abs(d(k));
%     else
%         F(k) = 0;
%     end
% end
% figure(1)
% plot(d,abs(F))
% xlabel('d')
% ylabel('|F|')
% 
% %% Foa
% rsav = 3;
% Koa = 30;
% d = 0.01:0.1:10;
% N = length(d);
% 
% for k=1:N
%     if d(k) < rsav
%         F(k) = ( (Koa/abs(d(k)))-Koa/rsav ) * d(k)/abs(d(k));
%     else
%         F(k) = 0;
%     end
% end
% figure(2)
% plot(d,abs(F))
% xlabel('d')
% ylabel('|F|')

%% Formation generation
% Eksempel med to skibe

% Antal skibe i formation
N = 2;
i = 1;%:N;
%j = 1:N;

% Sikkerhedsradius
rsav = 3;

% Virtuel leader pos
vl = [0,0];

% Desired pos af skibe
p0 = [2,2];
%p0(2) = [6,2];

% Pos af skibe
p = [6,4];
%p(2) = [6,6];

% Dists fra skibe til desired givet ud fra vl
d = vl - p;
d0 = vl - p0;
de = d - d0;

% Kraft mellem skib i og vl
Kvl = 1;
Fvl = Kvl*(de);

% % Dist mellem skibe
% d12 = p1 - p2;
% d12_0 = p1_0 - p2_0;
% d12_e = d12 - d12_0;
% 
% % Tiltrækning mellem skib i og j, minimerer imod dij0
Kij = 0.2;
% Fij_12 = Kij*(d0(i,j));
% 
% % Frastødning mellem skibe
Kca = 200;
% for k=1:N
%     if d12(k) < rsav
%         Fca(k) = ( (Kca*rsav)/abs(d12(k))-Kca ) * d12(k)/abs(d12(k));
%     else
%         Fca(k) = 0;
%     end
% end
% 
% 
% 
% % Frastødning mellem skibe og objekter
Koa = 200;
% 
% % Placering af objekt
% po1 = [4,4];
% 
% % Afstande mellem skibe til objekt
% do1 = p1 - po1;
% do2 = p2 - po1;
% 
% for k=1:N
%     if d(k) < rsav
%         F(k) = ( (Koa/abs(d(k)))-Koa/rsav ) * d(k)/abs(d(k));
%     else
%         F(k) = 0;
%     end
% end
% 
% %Fi = Fvl + Fij + Fca + Foa;


%% 3D plot with force magnitude

step = 0.5;
[X,Y] = meshgrid(-20:step:20,-20:step:20);
Ftot = zeros(length(X), length(Y),2);


for m = 1:length(X);
    for n = 1:length(Y);
        p = [X(1,m),Y(n,1)];
        Fvl = Kvl*(p-vl-(p0-vl));
        Fvlmagn(m,n) = norm(Fvl);
        
        pj = [4 , 4];
        dist = p - pj;
        d0ij = p0 - pj;
        Fij = Kij*(dist-d0ij);
        Fijmagn(m,n) = norm(Fij);
        
        if (dist) < rsav
            Fca = ((Kca*rsav)/norm(dist)-Kca)*(dist/norm(dist));
        else
            Fca = 0;
        end
        
        po1 = [-3,-3];
        dist = p - po1;
        if (dist) < rsav
            Foa = (Koa/norm(dist)-Koa/rsav)*(dist/norm(dist));
        else
            Foa = 0;
        end
        
        Ftot = Fvl+Fij+Fca+Foa;
        Ftotmagn(m,n) = norm(Ftot);
    end
end

% for k=1:N
%     if d(k) < rsav
%         F(k) = ( (Koa/abs(d(k)))-Koa/rsav ) * d(k)/abs(d(k));
%     else
%         F(k) = 0;
%     end
% end

figure(1)
clf;
hold on
axis equal
[xvel,yvel] = gradient(-Fvlmagn,step,step);
contour(X, Y, Fvlmagn);
quiver(X(1,:),Y(:,1),xvel,yvel);
hold off
figure(2)
clf;
hold on
surf(X, Y, Fvlmagn);
contour(X, Y, Fvlmagn);
hold off
figure(3)
hold on
surf(X, Y, Fijmagn);
hold off
figure(5)
Fcamagn(m,n) = norm(Fca);
surf(X,Y,Fcamagn)
figure(6)
Foamagn(m,n) = norm(Foa);
surf(X,Y,Foamagn)
figure(7)
surf(X,Y,Ftotmagn);










