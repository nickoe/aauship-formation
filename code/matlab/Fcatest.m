clear all; 

%% 3D plot with force magnitude

step = 0.2;
[X,Y] = meshgrid(-20:step:20,-20:step:20);
Ftot = zeros(length(X), length(Y),2);
rsav = 5;
vl = [10,10];
p0 = [2,2];
Kvl = 1;
Kij = 0.1;
Kca = 20;
Koa = 20;

for m = 1:length(X);
    for n = 1:length(Y);
        p = [X(1,m),Y(n,1)];
        Fvl = Kvl*(vl-p-(vl-p0));
        Fvlmagn(m,n) = norm(Fvl);
        
        pj = [4 , 4];
        dist = p - pj;
        d0ij = p0 - pj;
        Fij = Kij*(dist-d0ij);
        Fijmagn(m,n) = norm(Fij);
        
        if norm(dist) < rsav
            Fca = ((Kca*rsav)/norm(dist)-Kca)*(dist/norm(dist));
        else
            Fca = 0;
        end
        Fcamagn(m,n) = norm(Fca);
        
        po1 = [-6,-6];
        dist = p - po1;
        if norm(dist) < rsav
            Foa = (Koa/norm(dist)-Koa/rsav)*(dist/norm(dist));
        else
            Foa = 0;
        end
        Foamagn(m,n) = norm(Foa);
        
        Fmax = 80;
        Ftotmagn(m,n) = Fvlmagn(m,n)+Fijmagn(m,n)+Fcamagn(m,n)+Foamagn(m,n);
        Ftotmagn(m,n) = min([norm(Ftotmagn(m,n)),Fmax])*Ftotmagn(m,n)/norm(Ftotmagn(m,n));
        
        if isnan(Ftotmagn(m,n))
            Ftotmagn(m,n)  = Fmax;
        end
    end
end

figure(1)
clf;
hold on
axis equal
[xvel,yvel] = gradient(-Fvlmagn,step,step);
contour(X, Y, Fvlmagn);
quiver(X(1,:),Y(:,1),xvel,yvel);
title('Contour and quiver plot of Fvl')
hold off
figure(2)
clf;
hold on
surf(X, Y, Fvlmagn);
contour(X, Y, Fvlmagn);
title('Fvlmagn')
hold off
figure(3)
clf;
hold on
surf(X, Y, Fijmagn);
title('Fijmagn')
hold off
figure(4)
clf;
surf(X,Y,Fcamagn)
title('Fcamagn')
figure(5)
clf;
surf(X,Y,Foamagn)
title('Foamagn')
figure(6)
clf;
surf(X,Y,Ftotmagn);
title('Ftotmagn')


