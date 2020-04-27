close all
clear all
clc

m_100;

global erle;

erle_variables;

%% Tiempo de Inicio/Fin en función del empuje
ch3 = (double(CTUN.data(:,5)'));
t_start = 1;

i = 1;
while (ch3(i) < 450) && ( i < size(ch3,2))

    t_start = t_start + 1;
    i = i + 1;
end
t_end = size(ch3,2);

i = 1;
while (ch3(end - i) < 450) && (i < size(ch3,2))
    t_end = t_end - 1;
    i = i + 1;
end

throttle_time = CTUN.data(:,1)';

%% Ángulos de inclinación
% ATT --> 3007 x 10
% AHR2 --> 3007 x 8
% EKF1 --> 3007 x 15
t_ATT = ATT.data(:,1);
tinds = t_ATT >= throttle_time(t_start) & t_ATT <= throttle_time(t_end);
att_time = t_ATT(tinds)';

roll_v = (ATT.data(tinds,4)')*(pi/180);
pitch_v = (ATT.data(tinds,6)')*(pi/180);
yaw_v = (ATT.data(tinds,8)')*(pi/180);

%% Velocidades de los motores
% RCOU --> 3007 x 14
% ESC PWM (1000-2000 us)
t_RCOU = RCOU.data(:,1);
tinds = t_RCOU >= throttle_time(t_start) & t_RCOU <= throttle_time(t_end);
RCOU_time = t_RCOU(tinds)';
ts_RCOU = median(diff(t_RCOU));% 0.1 (s)

m0_raw = RCOU.data(tinds,3)'; % FR
m1_raw = RCOU.data(tinds,4)'; % BL
m2_raw = RCOU.data(tinds,5)'; % FL
m3_raw = RCOU.data(tinds,6)'; % BR

for k = 1:size(RCOU_time,2)
    if k == 1
        W0(k,1) = 0;
        W1(k,1) = 0;
        W2(k,1) = 0;
        W3(k,1) = 0;
    end
    if k > 1
         W0(1,k) = 2.139e-5*W0(1,k-1) + 1*m1_raw(1,k-1)-1200;
         W1(1,k) = 2.139e-5*W1(1,k-1) + 1*m0_raw(1,k-1)-1200;
         W2(1,k) = 2.139e-5*W2(1,k-1) + 1*m2_raw(1,k-1)-1200;
         W3(1,k) = 2.139e-5*W3(1,k-1) + 1*m3_raw(1,k-1)-1200;
    end
end

%% Velocidades lineales
% NTUN --> 2899 x 12
% EKF1 --> 3007 x 15
% IMU --> 15037 x 13
% BARO --> 3008 x 6

t_NTUN = NTUN.data(:,1);
tinds = t_NTUN >= throttle_time(t_start) & t_NTUN <= throttle_time(t_end);
NTUN_time = t_NTUN(tinds)';

 ts_NTUN = median(diff(NTUN_time));% 0.1 (s)
 
 Xd_real = NTUN.data(:,9)'./100;
 Xdd_real = diff(Xd_real)./ts_NTUN;
 Xdd_real = Xdd_real(tinds);
 Xd_real = Xd_real(tinds);
 
 Yd_real = NTUN.data(:,10)'./100;
 Ydd_real = diff(Yd_real)./ts_NTUN;
 Ydd_real = Ydd_real(tinds);
 Yd_real = Yd_real(tinds);
 
 t_BARO = BARO.data(:,1);
tinds = t_BARO >= throttle_time(t_start) & t_BARO <= throttle_time(t_end);
BARO_time = t_BARO(tinds)';

ts_BARO = median(diff(BARO_time));% 0.1 (s)
 Z_real = BARO.data(:,3)';
 Zd_real = diff(Z_real)./ts_BARO;
 Zdd_real = diff(Zd_real)./ts_BARO;
 Zdd_real = Zdd_real(tinds);
 Zd_real = Zd_real(tinds);

%  t_EKF1 = EKF1.data(:,1);
% tinds = t_EKF1 >= throttle_time(t_start) & t_EKF1 <= throttle_time(t_end);
% EKF1_time = t_EKF1(tinds)';
% 
% ts_EKF1 = median(diff(EKF1_time));% 0.1 (s)
%  Zd_real = EKF1.data(:,6)';
%  Zdd_real = diff(Zd_real)./ts_EKF1;
%  Zdd_real = Zdd_real(tinds);
%  Zd_real = Zd_real(tinds);
 

t_IMU = IMU.data(:,1);
tinds = t_IMU >= throttle_time(t_start) & t_IMU <= throttle_time(t_end);
IMU_time = t_IMU(tinds)';

 ts_IMU = median(diff(IMU_time));% 0.02 (s)
 
 Xdd_real_b = IMU.data(:,6)';
 Xdd_real_b = Xdd_real_b(tinds);
 
 Ydd_real_b = IMU.data(:,7)';
 Ydd_real_b = Ydd_real_b(tinds);
 
 Zdd_real_b = IMU.data(:,8);
 Zdd_real_b = Zdd_real_b(tinds);
 


%% Actuaciones 
U1 = erle.Kt*(W0.^2+W1.^2+W2.^2+W3.^2);

%% Modelo
Xd_modelo = 0;
Yd_modelo = 0;
Zd_modelo = 0;
for i = 1:size(U1,2)
    
    Xdd_modelo(1,i) = (1/erle.m)*(-(cos(roll_v(i))*sin(pitch_v(i))*cos(yaw_v(i)) + sin(roll_v(i))*sin(yaw_v(i)))*U1(i) - erle.Kdx*Xd_modelo);
    Ydd_modelo(1,i) = (1/erle.m)*(-(cos(roll_v(i))*sin(pitch_v(i))*sin(yaw_v(i)) - sin(roll_v(i))*cos(yaw_v(i)))*U1(i) - erle.Kdy*Yd_modelo);
    Zdd_modelo(1,i) = (1/erle.m)*(-(cos(roll_v(i))*cos(pitch_v(i)))*U1(i) - 0.3*Zd_modelo) + erle.g;
    
    Xd_modelo = Xdd_modelo(1,i)*ts_RCOU + Xd_modelo;
    Yd_modelo = Ydd_modelo(1,i)*ts_RCOU + Yd_modelo;
    Zd_modelo = Zdd_modelo(1,i)*ts_RCOU + Zd_modelo;
    
%     [Xdd_modelo_b(i),Ydd_modelo_b(i),Zdd_modelo_b(i)] = rotateGFtoBF(Xdd_modelo(1,i),Ydd_modelo(1,i),Zdd_modelo(1,i),roll_v(i),pitch_v(i),yaw_v(i));
%        Xdd_modelo_b(i) = (cos(pitch_v(i))*cos(yaw_v(i))*Xdd_modelo(i)) + (cos(pitch_v(i))*sin(yaw_v(i))*Ydd_modelo(i)) - sin(pitch_v(i))*Zdd_modelo(i);
    for j = 1:size(IMU_time,2)
        if RCOU_time(i) == IMU_time(j)
            [XddG_real_IMU(i),YddG_real_IMU(i),ZddG_real_IMU(i)] = rotateBFtoGF(Xdd_real_b(j),Ydd_real_b(j),Zdd_real_b(j),roll_v(i),pitch_v(i),yaw_v(i));
%             XddB_Real(i) =  Xdd_real_b(j);
        end
    end
    
end



figure();
plot(NTUN_time,Xdd_modelo);
hold on;
plot( NTUN_time, Xdd_real);
hold on;
plot( NTUN_time, -XddG_real_IMU);
legend('Modelo','NTUN','IMU');
xlabel('t(s)');ylabel('Xd (m/s^2)');
str = sprintf('Aceleración en XG (Kt = %.2f Kdx = %.2f)',erle.Kt,erle.Kdx);
title(str);

figure();
plot(NTUN_time,Ydd_modelo);
hold on;
plot( NTUN_time, Ydd_real);
hold on;
plot( NTUN_time, -YddG_real_IMU);
legend('Modelo','Real','IMU');
xlabel('t(s)');ylabel('Yd (m/s^2)');
str = sprintf('Aceleración en YG (Kt = %.2f Kdx = %.2f)',erle.Kt,erle.Kdy);
title(str);
% 
% figure();
% plot(EKF1_time,Zdd_modelo);
% hold on;
% plot( EKF1_time, Zdd_real);legend('Modelo','Real');
% xlabel('t(s)');ylabel('Zd (m/s^2)');
% str = sprintf('Aceleración en ZG (Kt = %.2f Kdx = %.2f)',erle.Kt,erle.Kdz);
% title(str);

figure();
plot(BARO_time,Zdd_modelo);
hold on;
plot( BARO_time, ZddG_real_IMU+erle.g);legend('Modelo','Real');
xlabel('t(s)');ylabel('Zd (m/s^2)');
str = sprintf('Aceleración en ZG (Kt = %.2f Kdx = %.2f)',erle.Kt,erle.Kdz);
title(str);

% figure();
% plot(NTUN_time,-XddG_real,NTUN_time,Xdd_modelo);legend('IMU','Modelo');title('Velocidad en XG (m/s^2)');
% figure();
% plot(NTUN_time,-YddG_real,NTUN_time,Ydd_modelo);legend('IMU','Modelo');title('Velocidad en YG (m/s^2)');
% figure();
% plot(NTUN_time,ZddG_real,NTUN_time,Zdd_modelo);legend('IMU','Modelo');title('Velocidad en ZG (m/s^2)');
% figure();
% plot(NTUN_time,XddB_Real,NTUN_time,Xdd_modelo_b);legend('IMU','Modelo');








