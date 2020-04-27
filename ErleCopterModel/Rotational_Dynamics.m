close all
clear all
clc

m_100;

global erle;

erle_variables;

%% Tiempo de Inicio/Fin en funci�n del empuje
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

%% �ngulos de inclinaci�n
% ATT --> 3007 x 10
% AHR2 --> 3007 x 8
% EKF1 --> 3007 x 15
t_ATT = ATT.data(:,1);
tinds = t_ATT >= throttle_time(t_start) & t_ATT <= throttle_time(t_end);
ATT_time = t_ATT(tinds)';
ts_ATT = median(diff(ATT_time));% 0.1 (s)
 
roll_v = (ATT.data(:,4)')*(pi/180);
pitch_v = (ATT.data(:,6)')*(pi/180);
yaw_v = (ATT.data(:,8)')*(pi/180);

rv = diff(roll_v)./ts_ATT;
ra = diff(rv)./ts_ATT;

pv = diff(pitch_v)./ts_ATT;
pa = diff(pitch_v)./ts_ATT;

yv = diff(yaw_v)./ts_ATT;
ya = diff(yaw_v)./ts_ATT;

% Escalamos seg�n el tiempo
roll_v = roll_v(tinds);
pitch_v = pitch_v(tinds);
yaw_v = yaw_v(tinds);

rv = rv(tinds);
ra = ra(tinds);

pv = pv(tinds);
pa = pa(tinds);

yv = yv(tinds);
ya = ya(tinds);

%% Velocidad de rotaci�n en (rad/s)
p = rv - yv.*sin(pitch_v);
q = pv.*cos(roll_v) + yv.*(sin(roll_v).*cos(pitch_v));
r = pv.*(-sin(roll_v)) + yv.*(cos(roll_v).*cos(pitch_v));

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

%% Actuaciones 
U1 = erle.Kt*(W0.^2+W1.^2+W2.^2+W3.^2);
U2 = (sqrt(2)/2) * erle.l * erle.Kt * (-W0.^2 + W1.^2 + W2.^2 - W3.^2);
U3 = (sqrt(2)/2) * erle.l * erle.Kt * (W0.^2 - W1.^2 - W2.^2 + W3.^2);
U4 = erle.Kd * (W0.^2 + W1.^2 - W2.^2 - W3.^2);
Om = W0+W1-W2-W3;

rv_sum = 0;
roll_sum = 0;
for i = 1:size(RCOU_time,2)
    ra_modelo = (1/erle.Ixx) * ((erle.Iyy - erle.Izz)*q(i)*r(i) - erle.Jr*q*Om(i) + U2(i));
    pa_modelo = (1/erle.Iyy) * ((erle.Izz - erle.Ixx)*p(i)*r(i) + erle.Jr*p*Om(i) + U3(i));
    ya_modelo = (1/erle.Izz) * ((erle.Ixx - erle.Iyy)*q(i)*p(i)  + U4(i));
    
    rv_sum = ra_modelo(i) * ts_RCOU + rv_sum;
    roll_sum = rv_sum * ts_RCOU + roll_sum;
    
    roll_modelo(i) = roll_sum;
    
    
end

figure();
plot(RCOU_time,ra,RCOU_time,ra_modelo);
legend('Real','Modelo');

figure();
plot(RCOU_time,pa,RCOU_time,pa_modelo);
legend('Real','Modelo');

% figure();
% plot(RCOU_time,ya,RCOU_time,ya_modelo);
% legend('Real','Modelo');













