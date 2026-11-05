clc; 
clearvars; 
close all;

%% Calculo de matrices de transformacion homogenea

syms th1 th2 th3 th4 d1 d2 a1 a2 a3 a4 a5 dx_tcp dy_tcp dz_tcp  real

A1=DenavitH(th1,         d1, a1,         -sym(pi)/2);
A2=DenavitH(th2,          0, a2,                  0);
A3=DenavitH(th3,          0, a3,                  0);
A4=DenavitH(-(th2+th3),  -d2, a4,          -sym(pi)/2);

TCP = [1 0 0 0;
       0 1 0 0;
       0 0 1 dz_tcp;
       0 0 0 1];

T4=simplify(A1*A2*A3*A4*TCP);
T3=simplify(A1*A2*A3);
T2=simplify(A1*A2);
T1=simplify(A1);

T5 = simplify(A3*TCP);
T_TCP = simplify(T3*TCP);

disp(T4)
disp(T_TCP)

disp(T3)







% disp('--- Matriz T_0_4 Simplificada ---');
% disp(T_0_4)


% %% Calculo de los jacobianos
% 
% z0_0 = [0; 0; 1];  
% p0_0 = [0; 0; 0];  
% 
% % --- Vectores Z del Jacobiano ---
% 
% z0_1= T_0_1(1:3, 3);
% z0_2= T_0_2(1:3, 3);
% z0_3= T_0_3(1:3, 3);
% 
% % --- Vectores de posicion ---
% 
% p0_1= T_0_1(1:3, 4);
% p0_2= T_0_2(1:3, 4);
% p0_3= T_0_3(1:3, 4);
% p0_4= T_0_4(1:3, 4);
% 
% % --- Armado de los jacobianos ---
% 
% J1= simplify([ cross(z0_0, p0_4 - p0_0) ; z0_0]);
% J2= simplify([ cross(z0_1, p0_4 - p0_1) ; z0_1]);
% J3= simplify([ cross(z0_2, p0_4 - p0_2) ; z0_2]);
% J4= simplify([ cross(z0_3, p0_4 - p0_3) ; z0_3]);
% J= [J1, J2, J3, J4];
% 
% disp('--- Jacobiano del manipulador ---');
% disp(J)
% 
% Tc = [ 1,  0,  0;  
%        0,  1,  0;   
%        0, -1,  1;   
%        0,  0, -1 ]; 
% 
% J_real = simplify(J * Tc);
% 
% disp('--- Jacobiano Real (Manipulador Paletizador 3 GDL) ---');
% disp(J_real)
% 
% 
% %% Declaracion de las masas y tensores de inercia
% 
% % --- ESLABÓN 1 (Base) ---
% 
% m1= 3950.91 / 1000; 
% 
% lc1= [-28.43; -1.47; 54.41] / 1000; 
% 
% I1_g_mm2 = [ 50119077.51,   129333.81,  7485448.24;
%                129333.81, 46461761.90,   -12835.41;
%               7485448.24,   -12835.41, 77973252.01 ];
% 
% I1= I1_g_mm2 * 1e-9;
% 
% % --- ESLABÓN 2 (Hombro) ---
% 
% m2 = 753.76 / 1000; 
% 
% 
% lc2= [-75.35; -49.10; 14.53] / 1000; 
% 
% I2_g_mm2 = [ 2772337.38,  2131723.00,   832376.34;
%              2131723.00,  4986738.58,   451762.89;
%               832376.34,   451762.89,  5964201.56 ];
% 
% I2 = I2_g_mm2 * 1e-9;
% 
% % --- ESLABÓN 3 (Antebrazo y Varilla Superior) ---
% 
% m3 = 1147.79 / 1000; 
% 
% lc3 = [-39.41; 108.68; -14.94] / 1000; 
% 
% I3_g_mm2 = [ 13158646.01, -1298090.11,  1837277.04;
%             -1298090.11,  7555681.09,  -494039.48;
%              1837277.04,  -494039.48, 18628411.66 ];
% 
% I3 = I3_g_mm2 * 1e-9;
% 
% % --- ESLABÓN 4 (Muñeca / Herramienta Final) ---
% 
% m4 = 113.31 / 1000; 
% lc4 = [41.35; -5.32; -0.37] / 1000; 
% 
% I4_g_mm2 = [ 187053.40,   19440.48,      66.14;
%               19440.48,  134220.32,     425.34;
%                  66.14,     425.34,  174271.54 ];
% 
% I4 = I4_g_mm2 * 1e-9;
% 
% %% Construccion de la matriz de inercia
% 
% % --- Velocidades lineales ---
% 
% Jv1= [J_real(1:3,1), zeros(3,2)];
% Jv1T= Jv1';
% 
% Jv2= [J_real(1:3, 1:2), zeros(3,1)];
% Jv2T= Jv2';
% 
% Jv3= J_real(1:3, 1:3);
% Jv3T= Jv3';
% 
% % --- Velocidades angulares ---
% 
% Jw1= [J_real(4:6,1), zeros(3,2)];
% Jw1T= Jw1';
% 
% Jw2= [J_real(4:6,1:2), zeros(3,1)];
% Jw2T= Jw2';
% 
% Jw3= J_real(4:6, 1:3);
% Jw3T= Jw3';
% 
% % --- Matrices de rotacion ---
% 
% R1 = T_0_1(1:3, 1:3);
% R2 = T_0_2(1:3, 1:3);
% R3 = T_0_3(1:3, 1:3);
% 
% R1T=R1';
% R2T=R2';
% R3T=R3';
% 
% % --- Computacion de la matriz M ---
% 
% M1= m1*Jv1T*Jv1+Jw1T*R1*I1*R1T*Jw1;
% M1q=simplify(M1);
% M2= m2*Jv2T*Jv2+Jw2T*R2*I2*R2T*Jw2;
% M2q=simplify(M2);
% M3= m3*Jv3T*Jv3+Jw3T*R3*I3*R3T*Jw3;
% M3q=simplify(M3);
% 
% MT=M1q+M2q+M3q;
% 
% 
% disp(' --- Computacion de la matriz M ---')
% 
% Mq=simplify(MT);
% disp(Mq)

