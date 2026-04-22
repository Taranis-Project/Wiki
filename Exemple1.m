clc 
%clear all
close all

format long

Periode_controleur = 30e-3; %Vitesse de la boucle numérique à valider lors du test

kv=210;                 %Kv tr/min pour 1 V
%Voltage_min=14.6;       %Voltage min batterie
%Voltage_max=16.8;       %Voltage max batterie

%Rpm_max=Voltage_min*kv;     %Formule théorique de la vitesse maximale
%w_max=Rpm_max*2*pi/60;      %Vitesse de rotation max du volant

Massevolumiqued=7800;   %Masse volumique de l'acier
Massevolumiquef=2700;   %Masse volumique de l'aluminium
MassevolumiqueLipo=534; %Masse volumique du Lithium ion

Dfext=6e-2;             %Diamétre extérieur du tube de la fuseX
Dfint=5.5e-2;           %Diamétre intérieur du tube de la fuseX
Lf=1.9;                 %Longeur de la fusée

Ddext=5.4e-2;           %Diamétre extérieur volant d'inertie
Ddint=2e-2;             %Diamétre intérieur volant d'inertie
Ld=0.097;               %Longeur du volant d'inertie

Largb=0.05;             %Largeur des batteries
Longb=0.2;              %Longeur des batteries

Dmext=5.4e-2;           %Diamétre extérieur bague ajout de masse section 1
Dmint=3.8e-2;           %Diamétre extérieur bague ajout de masse section 1
Lm1=0.018;              %Longeur bague ajout de masse section 1
Dm=5e-2;                %Diamétre extérieur bague ajout de masse section 2
Lm2=0.07;               %Longeur bague ajout de masse section 2

Troulis=21.6*0.8-2.5;   %Temps de la phase de contrôle roulis (interdis avant et après par Planète Science)

Massed=Massevolumiqued*pi*(Ddext^2-Ddint^2)/4*Ld; %Masse du volant d'inertie
Massef=Massevolumiquef*pi*(Dfext^2-Dfint^2)/4*Lf; %Masse du corps de la fusée
Massebatteries=MassevolumiqueLipo*Longb*Largb^2;  %Masse des batteries
Masse_ajout1=Massevolumiqued*pi*(Dmext^2-Dmint^2)/4*Lm1;    %Masse de la bague d'ajout de masse
Masse_ajout2=Massevolumiqued*pi*(Dm^2)/4*Lm2;
Masse_ailerons=0.568;           %Masse des ailerons réel (supposé ponctuel)

Massetot_reel=6.400;            %Masse réel de la fusée en kg
Massetotp=(Massed+Massef+Massebatteries+Masse_ajout1+Masse_ajout2+Masse_ailerons)/Massetot_reel   %On estime le pourcentage de la masse prise en compte pour l'étude de l'inertie.
                       
Bras_ailerons=1/3*75e-3+Dfext;                  %Bras de levier des ailerons

Jd=Massed*(Ddext^2-Ddint^2)/2/4;                %Moment d'inertie du volant d'inertie
Jf=Massef*(Dfext^2-Dfint^2)/2/4+4*Masse_ailerons*Bras_ailerons+Massebatteries*Largb^2/24+Masse_ajout1*(Dmext^2-Dmint^2)/2/4+Masse_ajout2*(Dm^2)/2/4;       %moment d'inertie de la fusée

Jw = Jd;                % kg.m2 moment duinertie du volant d'inertie.
Js = (Jf+Jd)*1/Massetotp;             % kg.m2 moment d'inertie de la fusée incluant le volant d'inertie (corriger via le pourcentage de la masse, du aux parachutes/système éléctronique, moteur etc..).
Jeq = Jw*Js/(Jw+Js);    % kg.m2 
Bw = 1e-6;              % N.m.s/rad frottements visqueux entre le volant et la fusée. (e-6 ou e-7 courant continue asynchrone)

A = [[0 1 0] ; [0 0 Bw/Js] ; [0 0 -Bw/Jeq] ];
B = [[0 0] ; [-1/Js 1/Js] ; [1/Jeq -1/Js] ];
C = [[0 180/pi 0] ; [0 0 9.55] ];                 %On prend en sortie la vitesse de rotation de la fusée mesurée (deg/s, valeur de notre capteur) et la vitesse de la roue mesurée (tr/min)
D = [[0 0] ; [0 0]];

sat_ss = ss(A,B,C,D);
sat_tf = tf(sat_ss);

G_cT = minreal(sat_tf(1,1)); %Fonction de transfert entre le couple moteur --> vitesse de rotation de la fusée
G_cW = minreal(sat_tf(2,1)); %Fonction de transfert entre le couple moteur --> vitesse de rotation du volant
G_dT = minreal(sat_tf(1,1)); %Fonction de transfert entre le couple de perturbation --> vitesse de rotation de la fusée
G_dW = minreal(sat_tf(2,1)); %Fonction de transfert entre le couple de perturbation --> vitesse de rotation du volant

G=G_cT/G_cW;                 %Fonction de transfert entre la vitesse de rotation du volant --> vitesse de rotation de la fusée

%On utilise SISOTOOL pour créer un correcteur

%sisotool(G);

s=tf('s');
Cor=(3*s^2+2*s+1)/s        %Correcteur continu (Intergrale)

Cor_c2d=c2d(Cor,Periode_controleur,'tustin')        %Conversion en numérique
G_c2d=c2d(G,Periode_controleur);

G_BO=G_c2d*Cor_c2d;                     %Boucle ouverte

G_CL=feedback(G_BO,1);                  %Boucle fermée

Kd = 4*s          % dérivateur
Kp = 3;
s = tf('s');
Kp_tf = tf(Kp)
Ki = 1/s     % intégrateur

Kd_z=c2d(Kd, Periode_controleur, 'tustin')
Kp_z=c2d(Kp_tf, Periode_controleur, 'tustin')
Ki_z=c2d(Ki, Periode_controleur, 'tustin')

Kd_z+Kp_z+Ki_z

figure(15)
t=[0:Periode_controleur:Troulis];
step(G_CL,t);
ylabel('\omega(t) (°)');xlabel('Temps (s)');
title('Coorection en vitesse avec I')
            
Constante_correcteur=-12.46;

conversion_voltage=16.4/1012;        %Relation entre le voltage réel de la batterie et la valeur du capteur

Constante=250/kv*conversion_voltage  %Constante du correcteur à diviser par la valeur de la batterie
