clear all; 

%% 3D plot with force magnitude

step = 0.1;
[X,Y] = meshgrid(-20:step:20,-20:step:20);
Ftot = zeros(length(X), length(Y),2);
rsav = 5;
vl = [10,10];
p0 = [2,2];
Kvl = 2;
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
        
        Fmax = 100;
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
density = 10;
[xvel,yvel] = gradient(-Fvlmagn(1:density:m,1:density:n),step,step);
contour(X, Y, Fvlmagn);
quiver(X(1:density:m,1:density:n), Y(1:density:m,1:density:n),xvel,yvel);
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
figure(7)
clf;
hold on
density = 4;
[totxvel,totyvel] = gradient(-Ftotmagn(1:density:m,1:density:n),step,step);
contour(X, Y, Ftotmagn);
quiver(X(1:density:m,1:density:n), Y(1:density:m,1:density:n),totxvel,totyvel);
clear m, clear n
m(1) = 40;
n(1) = 40;
for k = 1:1000;
    A = Ftotmagn(m(k)-1:m(k)+1,n(k)-1:n(k)+1);
    [minval,index] = min(A(:));
    [i,j] = ind2sub(size(A),index);
    m(k+1) = m(k)+(i-2);
    n(k+1) = n(k)+(j-2);
    xny(k) = X(m(k),n(k));
    yny(k) = Y(m(k),n(k));
end
plot(xny,yny)
plot(X(m(1),n(1)),Y(m(1),n(1)),'r*')
surf(X,Y,Ftotmagn-100);
axis equal
hold off









