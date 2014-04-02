clear all
for fighandle = findobj('Type','figure')', clf(fighandle), end

%% Data files
logpath = '/afs/ies.auc.dk/group/14gr1034/public_html/tests/';
testname = 'crashtest';
% fid = fopen('busroute/mbus5/gpsdata141212.txt'); % reduced GPS
% fidr = fopen('busroute/gpsdata141212.txt'); % all GPS
% kdata = load('busroute/mbus5/Kalmandata141212.txt'); % Position outputs
% adata = load('busroute/mbus5/accdata141212.csv'); % Accelerometer outputs

%% Data files
gps1file = fopen([logpath,testname,'/gps1.log']);
imudata = load([logpath,testname,'/imu.log']);


%% Reading GPS data
line = textscan(gps1file,'%f,%c,%f,%c,%f,%c,%f,%f');
time = line{1};
nmealat = line{3};
latsign = line{4};
nmealon = line{5};
lonsign = line{6};
nmeaspeed = line{7};
walltime = line{8};

%% Converts the latitude an longitide to decimal coordinates
[pos] = nmea2decimal({nmealat,latsign,nmealon,lonsign});
lat = pos(1,:);
lon = pos(2,:);

figure(1)
plot(lon,lat,'.r')
plot_google_map('maptype','satellite')
title('WGS84')


%%
latrad = lat*pi/180;
lonrad = lon*pi/180;
% hei = gpsdata(:,3);
N = length(lat);
hei=zeros(N,1);
x=zeros(N,1);
y=zeros(N,1);
z=zeros(N,1);
for kk = 1:N
    %[x(kk) y(kk) z(kk)] = wgs842ecef(latrad(kk),lonrad(kk),0);
    [x(kk) y(kk) z(kk)] = geodetic2ecef(latrad(kk),lonrad(kk),hei(kk),referenceEllipsoid('wgs84'));
end

%% Transform
%index = 4;
%meanlat = latrad(1);
%meanlon = lonrad(1);
 meanlat = 57.015179789287792*pi/180;
 meanlon = 9.985062449450744*pi/180;
meanhei = hei(1);
% [a b c]=wgs842ecef(meanlat,meanlon,meanhei);
[a b c]=geodetic2ecef(meanlat,meanlon,meanhei,referenceEllipsoid('wgs84'));
% plot3(a,b,c,'r*')
R_e2t = [-sin(meanlat)*cos(meanlon) -sin(meanlat)*sin(meanlon) cos(meanlat);...
    -sin(meanlon) cos(meanlon) 0;...
    -cos(meanlat)*cos(meanlon) -cos(meanlat)*sin(meanlon) -sin(meanlat)];

T = zeros(3,N);
for kk = 1:N
    T(:,kk) = R_e2t*([x(kk);y(kk);z(kk)]-[a;b;c]);
end
T = T';

figure(1)
% T(300:360,:) = 0
plot(T(:,2),T(:,1))
title('Raw GPS log (localframe)')



%%%%

%%
latradr = lat*pi/180;
lonradr = lon*pi/180;
% hei = gpsdata(:,3);
N = length(lat);
heir=zeros(N,1);
xr=zeros(N,1);
yr=zeros(N,1);
zr=zeros(N,1);
for kk = 1:N
    %[x(kk) y(kk) z(kk)] = wgs842ecef(latrad(kk),lonrad(kk),0);
    [xr(kk) yr(kk) zr(kk)] = geodetic2ecef(latradr(kk),lonradr(kk),heir(kk),referenceEllipsoid('wgs84'));
end

%% Transform
Tr = zeros(3,N);
for kk = 1:N
    Tr(:,kk) = R_e2t*([xr(kk);yr(kk);zr(kk)]-[a;b;c]);
end
Tr = Tr';


%% ADIS16405 Inertial Measurement Unit
supply = imudata(:,1);
xgyro = imudata(:,2);
ygyro = imudata(:,3);
zgyro = imudata(:,4);
xaccl = imudata(:,5);
yaccl = imudata(:,6);
zaccl = imudata(:,7);
xmagn = imudata(:,8);
ymagn = imudata(:,9);
zmagn = imudata(:,10);
temp = imudata(:,11);
aux_adc = imudata(:,12);

figure(4)
subplot(4,1,1)
plot(imudata(:,2:4))
title('Gyrometer')

subplot(4,1,2)
plot(imudata(:,5:7))
title('Accelerometer')

subplot(4,1,3)
plot(imudata(:,8:10))
title('Magnetometer')

subplot(4,1,4)
plot(imudata(:,11:12))
title('Supply, temperature and ADC of the ADIS16405 IMU')
