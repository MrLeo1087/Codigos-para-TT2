clc;
close all;
clearvars;

load('dinamica.mat');

M_c  = subs(M,  [a1, a2, a3, a4, m1, m2, m3, m4, xp0, xp1, xp3, xp4, yp0, yp1, yp3, yp4, zp1, zp3, zp4], ...
                [0.02774, 0.210, 0.23051, 0.08, 3.95, 0.62, 0.37, 0.42, -0.028, -0.15, -0.17, -0.07, -0.00147, 0.01, 0.01, -0.01, 0.02, -0.01, 0.05]);

C_c = subs(C, [a1, a2, a3, a4, m2, m3, m4, xp1, xp3, xp4, yp1, yp3, yp4, zp1, zp3, zp4], ...
              [0.02774, 0.210, 0.23051, 0.08, 0.62, 0.37, 0.42, -0.15, -0.17, -0.07, 0.01, 0.01, -0.01, 0.02, -0.01, 0.05]);

gq_c = subs(gq, [a2, a3, a4, g, m2, m3, m4, xp1, xp3, xp4, yp1, yp3, yp4], ...
                [0.210, 0.23051, 0.08, 9.81, 0.62, 0.37, 0.42, -0.15, -0.17, -0.07, 0.01, 0.01, -0.01]);

px1 = 0.3085; py1 = -0.00032; pz1 = 0.2846;
d1 = 0.100; a1 = 0.02774; a2 = 0.210; a3 = 0.23051; a4 = 0.42;
dx_tcp = 0.05034; dz_tcp = 0.0254; dy_tcp = -0.00032;

pos_home = [px1; py1; pz1];

m1 = 3.95; m2 = 0.62; m3 = 0.37; m4 = 0.42; g = 9.81; 

wpts = [pos_home(1)  -0.00032   -0.00032  -0.00032  pos_home(1)  0.00032  0.00032  0.00032  pos_home(1);
        pos_home(2)  -0.30808   -0.27494  -0.30808  pos_home(2)  0.30808  0.27494  0.30808  pos_home(2);
        pos_home(3)   0.28460    0.01553   0.28460  pos_home(3)  0.28460  0.01553  0.28460  pos_home(3)];

tpts = [0, 3, 6, 9, 12, 15, 18, 21, 24];
tvec = 0:0.001:24;

muestras = size(wpts,2);
wpts_angular = zeros(3, muestras);

for i=1:muestras

    [th1, th2, th3] = cinematica_inversa(wpts(1,i), wpts(2,i), wpts(3,i), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
    wpts_angular(1,i) = th1; 
    wpts_angular(2,i) = th2+pi; 
    wpts_angular(3,i) = th3-pi; 

end

[p_angular, d_angular, a_angular] = quinticpolytraj(wpts_angular,tpts,tvec);

p_angular = p_angular';
d_angular = d_angular';
a_angular = a_angular';

ts_q = timeseries(p_angular, tvec);
ts_dq = timeseries(d_angular, tvec);
ts_qddot = timeseries(a_angular, tvec);

matlabFunction(M_c,  'File', 'M_func',  'Vars', [theta2, theta3]);
matlabFunction(C_c,  'File', 'C_func',  'Vars', [theta2, theta3, dtheta1, dtheta2, dtheta3]);
matlabFunction(gq_c, 'File', 'gq_func', 'Vars', [theta2, theta3]);

N   = size(p_angular, 1);
tau = zeros(N, 3);

for i = 1:N
    Mi  = M_func( p_angular(i,2), p_angular(i,3));
    Ci  = C_func( p_angular(i,2), p_angular(i,3), d_angular(i,1), d_angular(i,2), d_angular(i,3));
    gi  = gq_func(p_angular(i,2), p_angular(i,3));
    
    % --- RUTINA DE CORRECCIÓN DINÁMICA DE TAMAÑO ---
    if numel(gi) == 9
        % Si gi llegó como matriz 3x3 (U_total no se sumó), sumamos las columnas
        gi_columna = sum(gi, 2); 
    elseif numel(gi) == 2
        % Si gi llegó como 2x1 (se omitió theta1), le agregamos el cero inicial
        gi_columna = [0; gi(:)];
    else
        % Si está correcto, solo nos aseguramos de que sea columna
        gi_columna = gi(:); 
    end
    % -----------------------------------------------
    
    % Ahora sí, sumamos tres vectores que son estrictamente de 3x1
    tau(i,:) = (Mi*a_angular(i,:)' + Ci*d_angular(i,:)' + gi_columna)';
end
figure
subplot(3,1,1); plot(tvec, tau(:,1), 'r', 'LineWidth', 1.5); ylabel('\tau_1 [Nm]'); 
subplot(3,1,2); plot(tvec, tau(:,2), 'g', 'LineWidth', 1.5); ylabel('\tau_2 [Nm]'); 
subplot(3,1,3); plot(tvec, tau(:,3), 'b', 'LineWidth', 1.5); ylabel('\tau_3 [Nm]'); 
xlabel('Tiempo (s)')
title('Torques dinámicos a lo largo de la trayectoria')



