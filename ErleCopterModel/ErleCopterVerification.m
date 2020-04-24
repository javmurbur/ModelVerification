close all
clear all
clc

m_100;

%% Find Start and End Times based on Thrust
ch3 = (double(CTUN.data(:,5)'));
t_start = 1;
t_end = size(RCIN.data,1);
t_pwm_in = (RCIN.data(:,1));

% El punto de estabilización esta sobre un throttle de 475
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

rpy_max = 45; % Ángulo máximo en grados

% Valores de entrada transformados
ch1 = -(double(RCIN.data(t_start:t_end,3)')-1450)*(rpy_max/400)*(2*pi/360); %Roll en rad
ch2 = -(double(RCIN.data(t_start:t_end,4)')-1500)*(rpy_max/400)*(2*pi/360); %Pitch en rad
ch3 = (double(CTUN.data(t_start:t_end,5)')- 475)/100; %Thrust 
ch4 = -(double(RCIN.data(t_start:t_end,6)')-1525)*(rpy_max/400)*(2*pi/360); %Yaw rate en rad/s

%% Salidas RC (PWM)

% ESC PWM (1000-2000 us)
m0_raw = RCOU.data(t_start:t_end,3)'; % FR
m1_raw = RCOU.data(t_start:t_end,1)'; % BL
m2_raw = RCOU.data(t_start:t_end,2)'; % FL
m3_raw = RCOU.data(t_start:t_end,3)'; % BR

%% Posición (m)
% Time (s)
t_NTUN = NTUN.data(:,1);
tinds = t_NTUN >= t_pwm_in(t_start) & t_NTUN <= t_pwm_in(t_end);
xy_pos_time = t_NTUN(tinds)';

Xg = NTUN.data(tinds,5)';
Yg = -NTUN.data(tinds,6)';

t_BARO = BARO.data(:,1);
tinds = t_BARO >= t_pwm_in(t_start) & t_BARO <= t_pwm_in(t_end);
z_pos_time = t_BARO(tinds)';

Zg = -BARO.data(tinds,3)'/100;

%% Ángulos de inclinacioón (rad)
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
rp = medianFilter(roll_v',40);  % rad
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

% Calculo las tasas de variación de ángulo
p = double(ones(size(rv,1),1).*rv) - double(yv.*sin(pp));
q = double(pv.*cos(rp)) + double(yv.*sin(rp).*cos(pp));
r = double(-pv.*sin(rp)) + double(yv.*cos(rp).*cos(pp));

% Caluclar aceleración de yaw
ch4_d = meanFilter(diff(ch4)',10)./ts_RC;
ch4_d = [0;ch4_d];



