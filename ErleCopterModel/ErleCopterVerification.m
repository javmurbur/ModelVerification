close all
clear all
clc

m_100;

addpath  C:\Users\javi\Desktop\Repositorios\ModelVerification\ErleCopterModel\rls\matlab
%% Find Start and End Times based on Thrust
ch3 = (double(CTUN.data(:,5)'));
t_start = 1;
t_end = size(RCIN.data,1);
t_pwm_in = (RCIN.data(:,1));

% El punto de estabilizaci�n esta sobre un throttle de 475
i = 1;
while((ch3(i) < 475) && (i < size(ch3,2)))
    t_start = t_start+1;
    i = i+1;
end

i = 1;
while((ch3(end-i))<475 && (i < size(ch3,2)))
    t_end = t_end-1;
    i = i+1;
end

rc_time = RCIN.data(t_start:t_end,1)';

%% Tiempo para las salidas PWM
t_pwm_out = RCOU.data(:,1);
tinds = t_pwm_out >= RCIN.data(t_start,1) & t_pwm_out <= RCIN.data(t_end);
pwm_out_time = t_pwm_out(tinds)';

%% Ajustar el tiempo para la IMU
t_imu = IMU.data(:,1);
tinds = t_imu >= RCIN.data(t_start,1) & t_imu <= RCIN.data(t_end,1);
imu_time = t_imu(tinds)';

%% IMU (m/s^2)
x_dd = IMU.data(tinds,6)';
y_dd = IMU.data(tinds,7)';
z_dd = IMU.data(tinds,8)';

%% Gyroscope (rad/s)
gyro_p =  IMU.data(tinds,3)';
gyro_q = -IMU.data(tinds,4)';
gyro_r = -IMU.data(tinds,5)';

%% Entradas RC (us)
ch1_raw = double(RCIN.data(t_start:t_end,3)');
ch2_raw = double(RCIN.data(t_start:t_end,4)');
ch3_raw = double(CTUN.data(t_start:t_end,5)');
ch4_raw = double(RCIN.data(t_start:t_end,6)');

rpy_max = 45; % �ngulo m�ximo en grados

% Valores de entrada transformados
ch1 = -(double(RCIN.data(t_start:t_end,3)')-1450)*(rpy_max/400)*(2*pi/360); %Roll en rad
ch2 = -(double(RCIN.data(t_start:t_end,4)')-1500)*(rpy_max/400)*(2*pi/360); %Pitch en rad
ch3 = (double(CTUN.data(t_start:t_end,5)')- 475); %Thrust 
ch4 = -(double(RCIN.data(t_start:t_end,6)')-1525)*(rpy_max/400)*(2*pi/360); %Yaw rate en rad/s

%% Salidas RC (PWM)

% ESC PWM (1000-2000 us)
m0_raw = RCOU.data(t_start:t_end,3)'; % FR
m1_raw = RCOU.data(t_start:t_end,4)'; % BL
m2_raw = RCOU.data(t_start:t_end,5)'; % FL
m3_raw = RCOU.data(t_start:t_end,6)'; % BR

%% Posici�n (m)
% Time (s)
t_NTUN = NTUN.data(:,1);
tinds = t_NTUN >= t_pwm_in(t_start) & t_NTUN <= t_pwm_in(t_end);
xy_pos_time = t_NTUN(tinds)';

Xg = NTUN.data(tinds,5)';
Yg = -NTUN.data(tinds,6)';

t_BARO = BARO.data(:,1);
tinds = t_BARO >= t_pwm_in(t_start) & t_BARO <= t_pwm_in(t_end);
z_pos_time = t_BARO(tinds)';

Zg = -BARO.data(tinds,3)';

%% �ngulos de inclinacio�n (rad)
t_ATT = ATT.data(:,1);
tinds = t_ATT >= t_pwm_in(t_start) & t_ATT <= t_pwm_in(t_end);
att_time = t_ATT(tinds)';

roll_v = (ATT.data(tinds,4)')/100*(pi/180);
pitch_v = (-ATT.data(tinds,6)')/100*(pi/180);
yaw_v = (-ATT.data(tinds,8)')/100*(pi/180);

%% Tiempos de muestreo
ts_RC = median(diff(RCIN.data(:,1)));
ts_NTUN = median(diff(NTUN.data(:,1)));% 0.1 (s)
ts_BARO = median(diff(BARO.data(:,1)));% 0.1 (s)
ts_ATT = median(diff(ATT.data(:,1)));% 0.1 (s)

%% Hacer que los vectores tengan las mismas dimensiones
rc_time = rc_time - rc_time(1);
imu_time = imu_time - rc_time(1);
xy_pos_time = xy_pos_time - rc_time(1);
z_pos_time = z_pos_time - rc_time(1);
att_time = att_time - rc_time(1);
pwm_out_time = pwm_out_time - rc_time(1);

%% Differentiate
% Eje X
gxv = meanFilter(diff(Xg)',40)./ts_NTUN; %(m/s)
gxa = meanFilter(diff(gxv),40)./ts_NTUN;
gxv = [0;gxv];
gxa = [0;0;gxa];
gxa([1:40,end-40:end]) = 0;

% Eje Y
gyv = meanFilter(diff(Yg)',40)./ts_NTUN; %(m/s)
gya = meanFilter(diff(gyv),40)./ts_NTUN;
gyv = [0;gyv];
gya = [0;0;gya];
gya([1:40,end-40:end]) = 0;

% Eje Z
gzv = meanFilter(diff(Zg)',40)./ts_BARO; %(m/s)
gza = meanFilter(diff(gzv),40)./ts_BARO;
gzv = [0;gzv];
gza = [0;0;gza];
gza([1:40,end-40:end]) = 0;

% Global to Body Rotations
bv = convertGlobalToBody([gxv,gyv,gza,yaw_v']);
bxv = bv(:,1); % m/s
byv = bv(:,2); % m/s

ba = convertGlobalToBody([gxa,gya,gza,yaw_v']);
bxa = ba(:,1); % m/s^2
bya = ba(:,2); % m/s^2

% Rotaciones
rp = medianFilter(roll_v',10);  % rad
rv = meanFilter(diff(rp),40)./ts_ATT; %rad/s
ra = meanFilter(diff(rv),40)./ts_ATT; %rad/s^2
rv = [0;rv];
ra = [0;0;ra];
ra([1:40,end-40:end]) = 0;

pp = medianFilter(pitch_v',40);  % rad
pv = meanFilter(diff(pp),40)./ts_ATT; %rad/s
pa = meanFilter(diff(pv),40)./ts_ATT; %rad/s^2
pv = [0;pv];
pa = [0;0;pa];
pa([1:40,end-40:end]) = 0;

yp = medianFilter(yaw_v',40);  % rad
yv = meanFilter(diff(yp),40)./ts_ATT; %rad/s
ya = meanFilter(diff(yv),40)./ts_ATT; %rad/s^2
yv = [0;yv];
ya = [0;0;ya];
ya([1:40,end-40:end]) = 0;

% Calculo las tasas de variaci�n de �ngulo
p = double(ones(size(rv,1),1).*rv) - double(yv.*sin(pp));
q = double(pv.*cos(rp)) + double(yv.*sin(rp).*cos(pp));
r = double(-pv.*sin(rp)) + double(yv.*cos(rp).*cos(pp));

% Caluclar aceleraci�n de yaw
ch4_d = meanFilter(diff(ch4)',10)./ts_RC;
ch4_d = [0;ch4_d];

%% Estimaci�n de los par�metros Motor
 Erle_KT = 8.5485e-6;
 Erle_Kd = 8.06428e-5;
 Erle_l = 0.141; %(m)
 
 % C�lculo de las velocidades utilizando las entradas a los ESC
 RCPer = median(diff(pwm_out_time));
 
 for k = 1:size(pwm_out_time,2)
     if k == 1
         w0(1,k) = 0;
         w1(1,k) = 0;
         w2(1,k) = 0;
         w3(1,k) = 0;
     end
     if k > 1
         w0(1,k) = 2.139e-5*w0(1,k-1) + 1*m1_raw(1,k-1)-1200;
         w1(1,k) = 2.139e-5*w1(1,k-1) + 1*m0_raw(1,k-1)-1200;
         w2(1,k) = 2.139e-5*w2(1,k-1) + 1*m2_raw(1,k-1)-1200;
         w3(1,k) = 2.139e-5*w3(1,k-1) + 1*m3_raw(1,k-1)-1200;
     end
 end
 
 w0 = w0 + 298.7;
 w1 = w1 + 298.7;
 w2 = w2 + 298.7;
 w3 = w3 + 298.7;
%  
%  w0 = w0(tinds);
%  w1 = w1(tinds);
%  w2 = w2(tinds);
%  w3 = w3(tinds);
 
 % C�lculo de las actuaciones
 U1 = Erle_KT*(w0.^2+w1.^2+w2.^2+w3.^2);
 U2 = (sqrt(2)/2)*Erle_Kd*Erle_l*(-w0.^2 + w1.^2 + w2.^2 - w3.^2);
%  U2 = Erle_KT*Erle_l*(w2.^2-w3.^2);
 U3 = (sqrt(2)/2)*Erle_KT*Erle_l*(-w0.^2 + w1.^2 - w2.^2 - w3.^2);
 U4 = Erle_Kd*(-w0.^2 - w1.^2 + w2.^2 + w3.^2);
 
 U2 = meanFilter(U2',2)';
 U3 = meanFilter(U3',2)';
 
%  % Comparar valores -> ESTA COMPARACI�N NO TIENE SENTIDO
%  temp = meanFilter(gyro_r',2);
%  plotyy(att_time,rp,pwm_out_time,U2');
%  legend('Y pos','U2','roll','RC');
%  
% 
%  figure();
%  plotyy(att_time,pa,pwm_out_time,-U3);
%  legend('X pos','U3','pitch','RC');
%  
%  figure();
%  plotyy(att_time,ya,pwm_out_time,-U4);
%  legend('Z pos','U4','yaw','RC');
%  

%% Alineaci�n temporal entre ATT y RCIN
offsets = linspace(-.5,.5,50);

RMSp = zeros(size(offsets));
RMSr = zeros(size(offsets));
RMSz = zeros(size(offsets));
RMSy = zeros(size(offsets));

i = 0;
for o = offsets
    i = i + 1;
    vinds = findLatestsInds(rc_time,att_time + o);
    
    ccf = corrcoef(double([pp(vinds),ch2']));
    RMSp(i) = ccf(1,2);
    
    ccf = corrcoef(double([rp(vinds),ch1']));
    RMSr(i) = ccf(1,2);
    
    ccf = corrcoef(double([-gza(vinds),ch3']));
    RMSz(i) = ccf(1,2);
    
    ccf = corrcoef(double([yv(vinds),ch4']));
    RMSy(i) = ccf(1,2);
    
end
 
 disp(' ');
 [v,oi] = max(RMSp);
 vindsp = findLatestsInds(rc_time,att_time + offsets(oi));
 disp(['Ardupilot/RC pitch angle delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSr);
 vindsr = findLatestsInds(rc_time,att_time + offsets(oi));
 disp(['Ardupilot/RC roll angle delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSz);
 vindsz = findLatestsInds(rc_time,att_time + offsets(oi));
 disp(['Ardupilot/RC thrust delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSy);
 vindsy = findLatestsInds(rc_time,att_time + offsets(oi));
 disp(['Ardupilot/RC yaw rate delay = ' num2str(offsets(oi)) 'sec']);
 disp(' ');
 
 %% Alineaci�n temporal entre ATT y ACTUACIONES
offsets = linspace(-.5,.5,50);

RMSp = zeros(size(offsets));
RMSr = zeros(size(offsets));
RMSz = zeros(size(offsets));
RMSy = zeros(size(offsets));

i = 0;
for o = offsets
    i = i + 1;
    vinds = findLatestsInds(pwm_out_time,att_time + o);
    
    ccf = corrcoef(double([-pa(vinds),U3']));
    RMSp(i) = ccf(1,2);
    
    ccf = corrcoef(double([ra(vinds),U2']));
    RMSr(i) = ccf(1,2);
    
    ccf = corrcoef(double([-gza(vinds),U1']));
    RMSz(i) = ccf(1,2);
    
    ccf = corrcoef(double([-ya(vinds),U4']));
    RMSy(i) = ccf(1,2);
    
end
 
 disp(' ');
 [v,oi] = max(RMSp);
 vindsU3 = findLatestsInds(pwm_out_time,att_time + offsets(oi));
 disp(['Ardupilot/ESC pitch acc delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSr);
 vindsU2 = findLatestsInds(pwm_out_time,att_time + offsets(oi));
 disp(['Ardupilot/ESC roll acc delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSz);
 vindsU1 = findLatestsInds(pwm_out_time,att_time + offsets(oi));
 disp(['Ardupilot/ESC z acc delay = ' num2str(offsets(oi)) 'sec']);
 
  [v,oi] = max(RMSy);
 vindsU4 = findLatestsInds(pwm_out_time,att_time + offsets(oi));
 disp(['Ardupilot/ESC yaw acc delay = ' num2str(offsets(oi)) 'sec']);
 disp(' ');
 
 %% Din�mica Translacional
 % Complejo
 global erle;
 erle_variables;
 
 Rx = (1/erle.m)*U1'.*(cos(rp(vindsU2)).*cos(yp(vindsU4)).*sin(pp(vindsU3)) + sin(rp(vindsU2)).*sin(yp(vindsU4)));
 Ax = [-Rx, -(1/erle.m)*0*gxv(vindsU3), ones(size(Rx,1),1)];
 Bx = gxa(vindsU3);
 [kxn,loosxn,bestlambdansxn] = lrlsloobest(Ax,Bx);
 
 Rz = (1/erle.m)*U1'.*(cos(rp(vindsU2)).*cos(pp(vindsU3)));
 Az = [-Rz, -(1/erle.m)*erle.Kdz*gzv(vindsU1), erle.g*ones(size(Rz,1),1)];
 Bz = gza(vindsU1);
 [kzn,looszn,bestlambdaszn] = lrlsloobest(Az, Bz);
 
 figure();
 
plot(pwm_out_time,[gxa(vindsU3),Ax*kxn])
legend('Measured','Model');

 figure();
 
plot(pwm_out_time,[gza(vindsU3),Az*kxn])
legend('Measured','Model');
 
% % Simple
% 
%  Ax=[-(1/erle.m)*pp(vindsp).*U1',ones(size(U1,2),1)];
%  Bx = bxa(vindsp);
%  [kx,loosx,bestlambdasx] = lrlsloobest(Ax, Bx);
% 
%  figure();
%  plot(rc_time,[bxa(vindsp),Ax*kx])
%  legend('Measured','Model');



