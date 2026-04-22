clc 
%clear all
close all

format long

K=-1.8479;
Tw=0.28017;
Zeta=0.36929;

s = tf('s');

f=K/(1+(2*Zeta*Tw)*s+(Tw*s)^2)

% Your PID definition
PID = -1.9379 * (1 + 0.48*s + 0.0576*s^2) / s;

Pe=0.01

H = tf([1 -1],[1 4 5],'InputDelay', 0.3); 
Hd = c2d(H,0.1,'foh');

PID_d = c2d(PID, Pe, 'tustin')

Kp=0.3*s/s
Ki=0.2/s
Kd=1*s

Kp_d=c2d(Kp, Pe, 'tustin')
Ki_d=c2d(Ki, Pe, 'tustin')
Kd_d=c2d(Kd, Pe, 'tustin')

figure(1)
step(-f)

figure(2)
step(CL)
