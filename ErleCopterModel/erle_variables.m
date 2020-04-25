global erle;

erle.Deg_Rad = pi/180;
erle.Rad_Deg = 180/pi;
%% Simulación
erle.T_simulacion = 5;%(segundos)
erle.T_escalon_roll = 0;%(segundios)
erle.T_escalon_pitch = 0;%(segundios)
erle.T_escalon_yaw = 0;%(segundios)
erle.Tm = 0.01;

%% Parámetros del motor
erle.Kt = 8.5486e-6;
erle.w_max = 838; % (rad/s)
erle.w_min = 0;
erle.Jr = 2.409e-4;

%% Parámetros físicos
erle.Kd = 8.06428e-5;
erle.Kdx = 0.5;
erle.Kdy = 0;
erle.Kdz = 0;
erle.l = 0.141; % (m)
erle.m = 1.12; % (Kg)
erle.Ixx = 0.0347563;% (Kg*m^2)
erle.Iyy = 0.0458929;% (Kg*m^2)
erle.Izz = 0.0977;   % (Kg*m^2)
erle.g = 9.81;

%% En coordenadas cuerpo
%% Posiciones
erle.X_BF = 0;
erle.Y_BF = 0;
erle.Z_BF = 0;
%% Velocidades lineales
erle.X_d_BF = 0;
erle.Y_d_BF = 0;
erle.Z_d_BF = 0;
%% Aceleraciones lineales
erle.X_dd_BF = 0;
erle.Y_dd_BF = 0;
erle.Z_dd_BF = 0;

%% Posiciones
erle.X = 0;
erle.Y = 0;
erle.Z = 0;
%% Velocidades lineales
erle.X_d = 0;
erle.Y_d = 0;
erle.Z_d = 0;
%% Aceleraciones lineales
erle.X_dd = 0;
erle.Y_dd = 0;
erle.Z_dd = 0;

%% Ángulos de inclinación
erle.roll = 0;
erle.pitch = 0;
erle.yaw = 0;
%% Variaciones de ángulo
erle.p = 0;
erle.q = 0;
erle.r = 0;
erle.p_ek_1 = 0;
erle.q_ek_1 = 0;
erle.r_ek_1 = 0;
%% Velocidades de los motores
erle.w0 = 0;
erle.w1 = 0;
erle.w2 = 0;
erle.w3 = 0;

%% Control de altura
%% Control de variación de ángulo
erle.Z_des = 0;
erle.Z_des_filt = 0;
erle.Z_des_filt_1 = 0;
% erle.Z = 0;
% erle.Z_d = 0;
% erle.Z_dd = 0;

% Control de variación del roll
% C = zpk([-10 -5],[0],2909.8);
%F = zpk([],[-1],1);
% [TI,TD,KP,KI,KD] = calculo_PID(2909.8,0.1,0.2)
% Control de altura
erle.Z_TI = 0.3000;
erle.Z_TI_F = 1;
erle.Z_TD = 0.0667;
erle.Z_KP = 872.9400;
erle.Z_KI = 2.9098e+03;
erle.Z_KD = 58.1960;
erle.U1_max = erle.Kt*4*erle.w_max^2;
erle.U1_min = 0;
erle.Z_KI_lim = 0;
erle.Z_Int_ek = 0;
erle.Z_ek_1 = 0;

%% Control de actitud
%Entradas 
erle.roll_des = 0;
erle.roll_des_filt = 0;
erle.roll_des_filt_1 = 0;
erle.pitch_des = 0;
erle.pitch_des_filt = 0;
erle.pitch_des_filt_1 = 0;
erle.yaw_des = 0;
erle.yaw_des_filt = 0;
erle.yaw_des_filt_1 = 0;

% Control del roll
% [TI,TD,KP,KI,KD] = calculo_PID(529.01,0.2,0.1);
% F_PID = tf([1],[0.2 1]);
erle.roll = 0;
erle.roll_TI = 0.2;
erle.roll_TD = 0;
erle.roll_TI_F = 0.25;
erle.roll_KP = 35.0800;
erle.roll_KI = 175.4000;
erle.roll_KD = 0;
erle.p_max = 45*erle.Deg_Rad;
erle.roll_Int_ek = 0;
erle.roll_ek_1 = 0;

% Control del pitch
% [TI,TD,KP,KI,KD] = calculo_PID(172.18,0.2,0);
% F_PID = tf([1],[0.25 1]);
erle.pitch_TI = 0.2;
erle.pitch_TD = 0;
erle.pitch_TI_F = 0.25;
erle.pitch_KP = 34.4360;
erle.pitch_KI = 172.1800;
erle.pitch_KD = 0;
erle.q_max = 45*erle.Deg_Rad;
erle.pitch_Int_ek = 0;
erle.pitch_ek_1 = 0;

% Control del yaw
% [TI,TD,KP,KI,KD] = calculo_PID(342.98,0.17,0);
% F_PID = tf([1],[0.5 1]);
erle.yaw_TI = 0.17;
erle.yaw_TD = 0;
erle.yaw_TI_F = 0.5;
erle.yaw_KP = 58.3066;
erle.yaw_KI = 342.9800;
erle.yaw_KD = 0;
erle.r_max = 90*erle.Deg_Rad;
erle.yaw_Int_ek = 0;
erle.yaw_ek_1 = 0;

% Salidas del control de actitud
erle.p_des = 0;
erle.q_des = 0;
erle.r_des = 0;

%% Control de variación de ángulo

% Control de variación del roll
% [TI,TD,KP,KI,KD] = calculo_PID(13.093,0.1,0)
erle.p_TI = 0.1;
erle.p_TD = 0;
erle.p_KP = 1.3093;
erle.p_KI = 13.0930;
erle.p_KD = 0;
erle.U2_max = sqrt(2)*erle.l*erle.Kt*erle.w_max^2;
erle.U2_min = -sqrt(2)*erle.l*erle.Kt*erle.w_max^2;
erle.p_Int_ek = 0;
erle.p_d = 0;

% Control de variación del pitch
% [TI,TD,KP,KI,KD] = calculo_PID(18.357,0.1,0);
erle.q_TI = 0.1;
erle.q_TD = 0;
erle.q_KP = 1.8357;
erle.q_KI = 18.3570;
erle.q_KD = 0;
erle.U3_max = sqrt(2)*erle.l*erle.Kt*erle.w_max^2;
erle.U3_min = -sqrt(2)*erle.l*erle.Kt*erle.w_max^2;
erle.q_Int_ek = 0;
erle.q_d = 0;

% Control de variación del yaw
% [TI,TD,KP,KI,KD] = calculo_PID(39.08,0.1,0);
erle.r_TI = 0.1;
erle.r_TD = 0;
erle.r_KP = 3.9080;
erle.r_KI = 39.0800;
erle.r_KD = 0;
erle.U4_max = 2*erle.Kd*erle.w_max^2;
erle.U4_min = -2*erle.Kd*erle.w_max^2;
erle.r_Int_ek = 0;
erle.r_d = 0;

% Salidas del rate control
erle.U1 = 0;
erle.U2 = 0;
erle.U3 = 0;
erle.U4 = 0;

%% Variables para graficar
erle.indice = 1;
erle.time_plot = [0:erle.Tm:erle.T_simulacion];

erle.U1_plot =   [0:erle.Tm:erle.T_simulacion];
erle.Z_des_plot = [0:erle.Tm:erle.T_simulacion];
erle.Z_d_plot = [0:erle.Tm:erle.T_simulacion];
erle.Z_plot = [0:erle.Tm:erle.T_simulacion];
erle.Z_dd_plot = [0:erle.Tm:erle.T_simulacion];
erle.Z_dd_BF_plot = [0:erle.Tm:erle.T_simulacion];

erle.p_plot =   [0:erle.Tm:erle.T_simulacion];
erle.U2_plot =   [0:erle.Tm:erle.T_simulacion];
erle.roll_des_plot = [0:erle.Tm:erle.T_simulacion];
erle.roll_plot = [0:erle.Tm:erle.T_simulacion];

erle.q_plot =   [0:erle.Tm:erle.T_simulacion];
erle.U3_plot =   [0:erle.Tm:erle.T_simulacion];
erle.pitch_des_plot = [0:erle.Tm:erle.T_simulacion];
erle.pitch_plot = [0:erle.Tm:erle.T_simulacion];

erle.r_plot =   [0:erle.Tm:erle.T_simulacion];
erle.U4_plot =   [0:erle.Tm:erle.T_simulacion];
erle.yaw_des_plot = [0:erle.Tm:erle.T_simulacion];
erle.ryaw_plot = [0:erle.Tm:erle.T_simulacion];

