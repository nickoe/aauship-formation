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
% Kij = 0.2;
% Fij_12 = Kij*(d0(i,j));
% 
% % Frastødning mellem skibe
% Kca = 200;
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
% Koa = 200;
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
[X,Y] = meshgrid(-10:step:10,-10:step:10);
Ftot = zeros(length(X), length(Y),2);


for m = 1:length(X);
    for n = 1:length(Y);
        p = [X(1,m),Y(n,1)];
        Fvl = Kvl*(p-vl-(p0-vl));
            for j = 1:N;
                pj = [X(1,m),Y(n,1)]
                p
                dij = p - pj;
                de(:) = dij;
                %d0ij = p0 - pj0;
                %Fij = Kij*(dij-d0ij);
            end
        
            
        
        Ftot(m,n,1:2) = Fvl;
        Ftotmagn(m,n) = norm(Fvl);
    end
end

figure(1)
% quiver(X,Y,Ftot(:,:,1),Ftot(:,:,2));
clf;
hold on
axis equal
[xvel,yvel] = gradient(-Ftotmagn,step,step);
contour(X, Y, Ftotmagn)
quiver(X(1,:),Y(:,1),xvel,yvel)
hold off
figure(2)
clf;
hold on
surf(X, Y, Ftotmagn);
contour(X, Y, Ftotmagn)
hold off











