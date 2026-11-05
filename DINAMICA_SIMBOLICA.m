clc;
clearvars;
close all;

syms theta1 theta2 theta3 real
syms xp0 yp0 zp0 xp1 yp1 zp1 xp3 yp3 zp3 xp4 yp4 zp4 xp5 yp5 zp5 real
syms d1 a1 a2 a3 a4 a5 real
syms m1 m2 m3 m4 m5 real
syms dtheta1 dtheta2 dtheta3 real
syms g real

theta3m = sym(pi) - theta2 - theta3;


%% Centro de masa 1
% xp0 = -0.028; yp0 = -0.00147; zp0 =  0.05441; % Validado con solidworks

% I0 = [ 0.05011908, -0.00748545,  0.00012933; %% Medido desde 01 theta2
%       -0.00748545,  0.07797325,  0.00001284;
%        0.00012933,  0.00001284,  0.04646176];

I0 =	[0.05011908  0.00012934  0.00748545;  %% Medido desde O0 theta1
	     0.00012934  0.04646176 -0.00001285;
	     0.00748545 -0.00001285  0.07797325];

% m1 = 3.95;

%% Centro de masa 2

% xp1 = -0.15; yp1 = 0.01; zp1 = 0.02; % Validado con solidworks

% Tensor de inercia local (Eslabón 2 / Hombro) 
% I1 = [ 0.00092938,  0.00018034, -0.00082251; %% Medido desde O2 theta 3
%        0.00018034,  0.00415113, -0.00009462;
%       -0.00082251, -0.00009462,  0.00341176];

I1 = [ 0.00415113 -0.00018034 -0.00009462;  %% Medido desde O1 theta2
	   0.00018034  0.00092938  0.00082251;
	  -0.00009462  0.00082251  0.00341176];
% 
% m2 = 0.62;

%% Centro de masa 3

% xp3 = -0.17; yp3 = 0.01; zp3 = -0.01;


% I2 = [0.00020108,  0.00011962,  0.00026713; %% Medido desde marco O3
%       0.00011962,  0.00361376,  0.00001305;
%       0.00026713,  0.00001305,  0.00351359];
  
I2 = [0.00361376 -0.00011961 -0.00001305; %% Medido desde O2 theta3
	 -0.00011961  0.00020108  0.00026713;
	 -0.00001305  0.00026713  0.00351359];

% m3 = 0.37;

%% Centro de masa 4

% xp4 = -0.07; yp4 = -0.01; zp4 = 0.05;

% I4 = [ 0.00024609,  0.00003111, -0.00009171;
%        0.00003111,  0.00048785, -0.00001517;
%       -0.00009171, -0.00001517,  0.00034122];

I4 = [0.00026466 -0.00007150 -0.00008754;
     -0.00007150  0.00046928  0.00003126;  %%Medido desde la rotacion de motor 3
     -0.00008754  0.00003126  0.00034122];

% m4 = 0.42;


%% Centro de masa 5 

% xp5 = -0.10; yp5 = -0.01; zp5 = -0.02;

I5 = [ 0.00000993,  0.00000000,  0.00000000;
       0.00000000,  0.00056854,  0.00000000;
       0.00000000,  0.00000000,  0.00057598];

% m5 = 0.089;

%% Medidas fisicas del robot

% d1 = 0.100;
% a1 = 0.02774;
% a2 = 0.210;
% a3 = 0.23051;
% a4 = 0.08;
% a5 = 0.210;

R1 = [ cos(theta1) -sin(theta1) 0;
       sin(theta1)  cos(theta1) 0;
                 0            0 1];

R2 = [cos(theta1)  0 -sin(theta1) a1*cos(theta1);
      sin(theta1)  0  cos(theta1) a1*sin(theta1);
                0 -1            0             d1;
                0  0            0              1];

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

% syms dtheta1 dtheta2 dtheta3 dtheta4 g

q    = [theta1; theta2; theta3];
dq   = [dtheta1; dtheta2; dtheta3];

Jv0 = jacobian(Cm0, q);
Jv1 = jacobian(Cm1, q);
Jv2 = jacobian(Cm2, q);
Jv3 = jacobian(Cm3, q);

v0 = Jv0 * dq;
v1 = Jv1 * dq;
v2 = Jv2 * dq;
v3 = Jv3 * dq;

kv0 = (1/2)*m1*(v0'*v0);
kv1 = (1/2)*m2*(v1'*v1);
kv2 = (1/2)*m3*(v2'*v2);
kv3 = (1/2)*m4*(v3'*v3);

z0 = R1(1:3,3);
z1 = R2(1:3,3);
z2 = R3(1:3,3);

w0 = dtheta1*z0;
w1 = w0+dtheta2*z1;
w2 = w1+dtheta3*z2;
w3 = w0+dtheta3*z1;

Rrot0 = R2(1:3,1:3);
Rrot1 = R3(1:3,1:3);   
Rrot2 = R4(1:3,1:3);   
Rrot3 = R5(1:3,1:3);

Tensor0 = Rrot0 * I0 * Rrot0';
Tensor1 = Rrot1 * I1 * Rrot1';
Tensor2 = Rrot2 * I2 * Rrot2';
Tensor3 = Rrot3 * I4 * Rrot3';

wv0 = (1/2)*w0'*Tensor0*w0;
wv1 = (1/2)*w1'*Tensor1*w1; 
wv2 = (1/2)*w2'*Tensor2*w2;
wv3 = (1/2)*w3'*Tensor3*w3;

kt0 = kv0+wv0;
kt1 = kv1+wv1;
kt2 = kv2+wv2;
kt3 = kv3+wv3;

KT = kt0 + kt1 + kt2 + kt3;

g_v = [0; 0; -g];

EP0 = m1 * g_v * Cm0(3,1);
EP1 = m2 * g_v * Cm1(3,1);
EP2 = m3 * g_v * Cm2(3,1);
EP3 = m4 * g_v * Cm3(3,1);

EPT = EP0 + EP1 + EP2 + EP3;

L = simplify(KT - EPT);

M   = simplify(jacobian(jacobian(KT, dq)', dq));
gq  = simplify(jacobian(EPT, q)');

C = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        for k = 1:3
            C(i,j) = C(i,j) + (1/2)*(diff(M(i,j),q(k)) + ...
                                      diff(M(i,k),q(j)) - ...
                                      diff(M(j,k),q(i)))*dq(k);
        end
    end
end

C = simplify(C);

save('dinamica.mat', 'M', 'C', 'gq', 'theta1', 'theta2', 'theta3', 'dtheta1', 'dtheta2', 'dtheta3', 'a1', 'a2', 'a3', 'a4', 'm1', 'm2', 'm3', 'm4', 'xp0', 'xp1', 'xp3', 'xp4', 'yp0', 'yp1', 'yp3', 'yp4', 'zp1', 'zp3', 'zp4','g');

