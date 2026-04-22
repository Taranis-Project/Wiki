clc
%clear all
close all

format long

Controller_period = 30e-3; %Speed of the digital loop to validate during the test

kv=210;                 %Kv rpm for 1 V
%Voltage_min=14.6;       %Minimum battery voltage
%Voltage_max=16.8;       %Maximum battery voltage

%Rpm_max=Voltage_min*kv;     %Theoretical formula for maximum speed
%w_max=Rpm_max*2*pi/60;      %Maximum rotational speed of the flywheel

Density_steel=7800;     %Density of steel
Density_aluminum=2700;  %Density of aluminum
Density_Lipo=534;       %Density of Lithium-ion

Rocket_outer_diameter=6e-2;      %Outer diameter of the rocket body tube
Rocket_inner_diameter=5.5e-2;    %Inner diameter of the rocket body tube
Rocket_length=1.9;               %Rocket length

Flywheel_outer_diameter=5.4e-2;  %Outer diameter of the flywheel
Flywheel_inner_diameter=2e-2;    %Inner diameter of the flywheel
Flywheel_length=0.097;           %Length of the flywheel

Battery_width=0.05;              %Battery width
Battery_length=0.2;              %Battery length

Massring1_outer_diameter=5.4e-2; %Outer diameter of added mass ring section 1
Massring1_inner_diameter=3.8e-2; %Inner diameter of added mass ring section 1
Massring1_length=0.018;          %Length of added mass ring section 1
Massring2_outer_diameter=5e-2;   %Outer diameter of added mass ring section 2
Massring2_length=0.07;           %Length of added mass ring section 2

Roll_control_time=21.6*0.8-2.5;  %Duration of the roll control phase (forbidden before and after by Planète Science)

Flywheel_mass=Density_steel*pi*(Flywheel_outer_diameter^2-Flywheel_inner_diameter^2)/4*Flywheel_length; %Flywheel mass
Rocket_body_mass=Density_aluminum*pi*(Rocket_outer_diameter^2-Rocket_inner_diameter^2)/4*Rocket_length; %Rocket body mass
Battery_mass=Density_Lipo*Battery_length*Battery_width^2;  %Battery mass
Added_mass1=Density_steel*pi*(Massring1_outer_diameter^2-Massring1_inner_diameter^2)/4*Massring1_length; %Added mass ring 1
Added_mass2=Density_steel*pi*(Massring2_outer_diameter^2)/4*Massring2_length;
Fin_mass=0.568;           %Actual fin mass (assumed point mass)

Total_mass_real=6.400;    %Actual rocket mass in kg
Mass_ratio=(Flywheel_mass+Rocket_body_mass+Battery_mass+Added_mass1+Added_mass2+Fin_mass)/Total_mass_real
%Estimated percentage of mass considered in the inertia study
                       
Fin_lever_arm=1/3*75e-3+Rocket_outer_diameter;  %Lever arm of the fins

Flywheel_inertia=Flywheel_mass*(Flywheel_outer_diameter^2-Flywheel_inner_diameter^2)/2/4; %Flywheel inertia
Rocket_inertia=Rocket_body_mass*(Rocket_outer_diameter^2-Rocket_inner_diameter^2)/2/4 ...
    +4*Fin_mass*Fin_lever_arm ...
    +Battery_mass*Battery_width^2/24 ...
    +Added_mass1*(Massring1_outer_diameter^2-Massring1_inner_diameter^2)/2/4 ...
    +Added_mass2*(Massring2_outer_diameter^2)/2/4; %Rocket inertia

Jw = Flywheel_inertia;        %Flywheel inertia
Js = (Rocket_inertia+Flywheel_inertia)*1/Mass_ratio; %Rocket inertia including flywheel
Jeq = Jw*Js/(Jw+Js);          %Equivalent inertia
Bw = 1e-6;                    %Viscous friction between wheel and rocket

A = [[0 1 0] ; [0 0 Bw/Js] ; [0 0 -Bw/Jeq] ];
B = [[0 0] ; [-1/Js 1/Js] ; [1/Jeq -1/Js] ];
C = [[0 180/pi 0] ; [0 0 9.55] ];  %Outputs: rocket rotation speed (deg/s) and wheel speed (rpm)
D = [[0 0] ; [0 0]];

state_space_system = ss(A,B,C,D);
transfer_functions = tf(state_space_system);

G_control_rocket = minreal(transfer_functions(1,1)); %Motor torque → rocket angular speed
G_control_wheel = minreal(transfer_functions(2,1));  %Motor torque → wheel speed
G_disturbance_rocket = minreal(transfer_functions(1,1)); %Disturbance torque → rocket speed
G_disturbance_wheel = minreal(transfer_functions(2,1));  %Disturbance torque → wheel speed

G = G_control_rocket/G_control_wheel; %Wheel speed → rocket rotation speed

%Use SISOTOOL to design a controller
%sisotool(G);

s=tf('s');
Controller=(3*s^2+2*s+1)/s;   %Continuous controller (integral)

Controller_d=c2d(Controller,Controller_period,'tustin'); %Digital conversion
G_d=c2d(G,Controller_period);

Open_loop=G_d*Controller_d;
Closed_loop=feedback(Open_loop,1);

Kd = 4*s;          %Derivative
Kp = 3;
s = tf('s');
Kp_tf = tf(Kp);
Ki = 1/s;          %Integrator

Kd_z=c2d(Kd, Controller_period, 'tustin')
Kp_z=c2d(Kp_tf, Controller_period, 'tustin')
Ki_z=c2d(Ki, Controller_period, 'tustin')

Kd_z+Kp_z+Ki_z

figure(15)
t=[0:Controller_period:Roll_control_time];
step(Closed_loop,t);
ylabel('\omega(t) (°)');
xlabel('Time (s)');
title('Speed control with I')

Controller_constant=-12.46;

Voltage_conversion=16.4/1012; %Relation between real battery voltage and sensor value

Constant=250/kv*Voltage_conversion %Controller constant divided by battery value
