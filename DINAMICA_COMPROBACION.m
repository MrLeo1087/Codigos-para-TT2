clc;
clearvars;
close all;

theta1 = -pi/2;
theta2 = -0.785398;
theta3 =  pi/2;

theta3m = theta3-((3*pi)/2);

% theta1 = 0;
% theta2 = -pi/2;
% theta3 = pi/2;


%% Centro de masa 1
xp0 = -0.028; yp0 = -0.00147; zp0 =  0.05441; % Validado con solidworks

I0 = [ 0.05011908, -0.00748545,  0.00012933;
      -0.00748545,  0.07797325,  0.00001284;
       0.00012933,  0.00001284,  0.04646176];

m1 = 3.95;

%% Centro de masa 2

xp1 = -0.15; yp1 = 0.01; zp1 = 0.02; % Validado con solidworks

% Tensor de inercia local (Eslabón 2 / Hombro)
I1 = [ 0.00092938,  0.00018034, -0.00082251;
       0.00018034,  0.00415113, -0.00009462;
      -0.00082251, -0.00009462,  0.00341176];

m2 = 0.62;

%% Centro de masa 3

xp3 = -0.17; yp3 = 0.01; zp3 = -0.01;


I2 = [0.00020108,  0.00011962,  0.00026713;
      0.00011962,  0.00361376,  0.00001305;
      0.00026713,  0.00001305,  0.00351359];

m3 = 0.37;

%% Centro de masa 4

xp4 = -0.07; yp4 = -0.01; zp4 = 0.05;

I4 = [ 0.00024609,  0.00003111, -0.00009171;
       0.00003111,  0.00048785, -0.00001517;
      -0.00009171, -0.00001517,  0.00034122];

m4 = 0.42;


%% Centro de masa 5 

xp5 = -0.10; yp5 = -0.01; zp5 = -0.02;

I5 = [ 0.00000993,  0.00000000,  0.00000000;
       0.00000000,  0.00056854,  0.00000000;
       0.00000000,  0.00000000,  0.00057598];

m5 = 0.089;

%% Medidas fisicas del robot

d1 = 0.100;
a1 = 0.02774;
a2 = 0.210;
a3 = 0.23051;
a4 = 0.08;
a5 = 0.210;

R1 = [ cos(theta1) -sin(theta1) 0;
       sin(theta1)  cos(theta1) 0;
                 0            0 1];

R3 = [cos(theta1)*cos(theta2) -cos(theta1)*sin(theta2) -sin(theta1) cos(theta1)*(a1 + a2*cos(theta2));
      cos(theta2)*sin(theta1) -sin(theta1)*sin(theta2)  cos(theta1) sin(theta1)*(a1 + a2*cos(theta2));
                 -sin(theta2)             -cos(theta2)            0               d1 - a2*sin(theta2);
                            0                        0            0                                1];

R4 = [cos(theta2 + theta3)*cos(theta1) -sin(theta2 + theta3)*cos(theta1) -sin(theta1) cos(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2));
      cos(theta2 + theta3)*sin(theta1) -sin(theta2 + theta3)*sin(theta1)  cos(theta1) sin(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2));
                 -sin(theta2 + theta3)             -cos(theta2 + theta3)            0               d1 - a3*sin(theta2 + theta3) - a2*sin(theta2);
                                     0                                 0            0                                                          1];

R5 = [cos(theta1)*cos(theta3m) -cos(theta1)*sin(theta3m) -sin(theta1) cos(theta1)*(a1 + a4*cos(theta3m));
      cos(theta3m)*sin(theta1) -sin(theta1)*sin(theta3m)  cos(theta1) sin(theta1)*(a1 + a4*cos(theta3m));
                 -sin(theta3m)             -cos(theta3m)            0               d1 - a4*sin(theta3m);
                             0                         0            0                                 1];

R6 = [cos(theta3m + theta3)*cos(theta1) -sin(theta3m + theta3)*cos(theta1) -sin(theta1) cos(theta1)*(a1 + a5*cos(theta3m + theta3) + a4*cos(theta3m));
      cos(theta3m + theta3)*sin(theta1) -sin(theta3m + theta3)*sin(theta1)  cos(theta1) sin(theta1)*(a1 + a5*cos(theta3m + theta3) + a4*cos(theta3m));
                 -sin(theta3m + theta3)             -cos(theta3m + theta3)            0               d1 - a5*sin(theta3m + theta3) - a4*sin(theta3m);
                                      0                                  0            0                                                            1];


P1 = [xp0; yp0; zp0];
P2 = [xp1; yp1; zp1; 1];
P3 = [xp3; yp3; zp3; 1];
P4 = [xp4; yp4; zp4; 1];
P5 = [xp5; yp5; zp5; 1];

Cm0 = (R1*P1); %Eslabon 1
C1 =  R3*P2;
Cm1 = C1(1:3,1); %Eslabon 2
C2 = R4*P3;
Cm2 = C2(1:3,1); %Eslabon 3
C3 = R5*P4;
Cm3 = C3(1:3,1); %Biela 
C4 = R6*P5;
Cm4 = C4(1:3,1); %Acoplador

%% Matrices de Rotacion 





