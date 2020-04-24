clear all
close all
clc

m_100;

t_start = 1;
t_end = size(RCOU.data,1);

time = RCOU.data(:,1);
PWM = RCOU.data(:,3);
RPM = (RCOU.data(:,3)-1000);
RPM = RPM*(60/pi);

RPMinds = RPM >= 2500;

time = time(RPMinds);
PWM = PWM(RPMinds);
RPM = RPM(RPMinds);

%% Modelo

tau = 0.0186;
DC = 0.993;

s = tf('s');
G = DC/(tau*s+1)

dt = 0.1;
t = time(1):dt:time(end);
t = t-time(1);

RPM_sim = lsim(G,PWM-1200,t);
RPM_sim = RPM_sim *(60/pi);
%% Modelo discreto

RCPer = median(diff(time));
sysD = c2d(G,0.2,'zoh');

for k = 1:size(time,1)
    if k == 1
        RPM_est(k,1) = 0;
    end
    if k > 1
        RPM_est(k,1) = (2.139e-05*RPM_est(k-1,1) + 1*PWM(k-1)-1200)*(60/pi);
    end
end

figure();
plot(time,RPM,'r');
hold on;
plot(time,RPM_est+3808.56,'k');
% plot(time,RPM_sim+3808.56,'k');
str = sprintf('Simulated vs Actual Motor Dynamics (DC = %.2f TC = %.2f)',DC,tau);
title(str);
xlabel('Time(s)');
ylabel('RPM');
legend('Actual','Simulated');










