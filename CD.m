clc;
clearvars;
close all;

%% Puntos de prueba

px1 = 0.3085;
py1 = -0.00032;
pz1 = 0.2846;

px2 = 0.00032;
py2 = 0.30808;
pz2 = 0.28460;

px3 = -0.00032;
py3 = -0.27494;
pz3 =  0.01553;

%% Diemensiones del robot

d1     = 0.100;
a1     = 0.02774;
a2     = 0.210;
a3     = 0.23051;

dx_tcp = 0.05034;
dy_tcp = 0.0254;
dz_tcp = -0.00032;

theta1 = 0;
theta2 = -pi/2;
theta3 = pi/2;

%% Matrices de transformacion homogenea

TCP = [1  0 0  dx_tcp;
       0 -1 0  dy_tcp;
       0  0 -1 dz_tcp;
       0  0 0  1];

% A1=DenavitH(theta1, d1, a1, -pi/2);
% A2=DenavitH(theta2, 0, a2, 0);
% A3=DenavitH(theta3, 0, a3, 0);
% 
% T3=A1*A2*A3;
% T2=A1*A2;
% T1=A1;
% 
% T4=T3*TCP;

%% Cienamtica directa

x = cos(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2)) - dz_tcp*sin(theta1) + dx_tcp*cos(theta2 + theta3)*cos(theta1) - dy_tcp*sin(theta2 + theta3)*cos(theta1);
y = sin(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2)) + dz_tcp*cos(theta1) + dx_tcp*cos(theta2 + theta3)*sin(theta1) - dy_tcp*sin(theta2 + theta3)*sin(theta1);
z = d1 - dy_tcp*cos(theta2 + theta3) - a3*sin(theta2 + theta3) - dx_tcp*sin(theta2 + theta3) - a2*sin(theta2);

%% Comprobacion de funciones

% [th1_p, th2_p, th3_p] = cinematica_inversa(px3, py3, pz3, d1, a1, a2, a3, dx_tcp, dy_tcp);
% [x_p, y_p, z_p] = cinematica_directa(th1_p, th2_p, th3_p, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);

pos_home = [px1;
            py1;
            pz1];

%%     POS SEGURA                RECOJE             REGRESA               DEPOSITA          REGRESA POS SEGURA

wpts = [pos_home(1)  -0.00032   -0.00032  -0.00032  pos_home(1)  0.00032  0.00032  0.00032  pos_home(1);
        pos_home(2)  -0.30808   -0.27494  -0.30808  pos_home(2)  0.30808  0.27494  0.30808  pos_home(2);
        pos_home(3)   0.28460    0.01553   0.28460  pos_home(3)  0.28460  0.01553  0.28460  pos_home(3)];

tpts = [0, 3, 6, 9, 12, 15, 18, 21, 24];
tvec = 0:0.001:24;

%% Interpolacion cartesiana

[p_cart, p_cart_vel, p_cart_acc, pp] = quinticpolytraj(wpts, tpts, tvec);
p_cart = p_cart';
samples = size(p_cart,1);
jointAngles = zeros(samples,3);

for i = 1:samples
    [th1, th2, th3] = cinematica_inversa(p_cart(i,1), p_cart(i,2), p_cart(i,3), d1, a1, a2, a3, dx_tcp, dy_tcp);
    jointAngles(i,1) = th1;
    jointAngles(i,2) = th2;
    jointAngles(i,3) = th3;
end

plot(tvec, jointAngles(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(tvec, jointAngles(:,2), 'g', 'LineWidth', 1.5);
plot(tvec, jointAngles(:,3), 'b', 'LineWidth', 1.5);
legend('\theta_1', '\theta_2', '\theta_3')
xlabel('Tiempo (s)')
ylabel('Ángulo (rad)')
title('Trayectoria Cartesiana - Cinemática Inversa Analítica')
grid on

figure
plot3(p_cart(:,1), p_cart(:,2), p_cart(:,3), 'b', 'LineWidth', 1.5)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Trayectoria cartesiana')
grid on
axis equal
view(45, 25)
hold on
scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 80, 'r', 'filled')

%% Interpolacion angular

muestras = size(wpts,2);
wpts_angular = zeros(3, muestras);

for i=1:muestras

    [th1, th2, th3] = cinematica_inversa(wpts(1,i), wpts(2,i), wpts(3,i), d1, a1, a2, a3, dx_tcp, dy_tcp);
    wpts_angular(1,i) = th1; 
    wpts_angular(2,i) = th2; 
    wpts_angular(3,i) = th3; 

end

[p_angular, d_angular, a_angular] = quinticpolytraj(wpts_angular,tpts,tvec);
p_angular = p_angular';
d_angular = d_angular';
a_angular = a_angular';

figure
plot(tvec, p_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(tvec, p_angular(:,2), 'g', 'LineWidth', 1.5);
plot(tvec, p_angular(:,3), 'b', 'LineWidth', 1.5);
legend('\theta_1', '\theta_2', '\theta_3')
xlabel('Tiempo (s)')
ylabel('Velocidad rads/s')
title('Velocidad articular Articular - Cinemática Inversa Analítica')
grid on

figure
plot(tvec, a_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(tvec, a_angular(:,2), 'g', 'LineWidth', 1.5);
plot(tvec, a_angular(:,3), 'b', 'LineWidth', 1.5);
legend('\theta_1', '\theta_2', '\theta_3')
xlabel('Tiempo (s)')
ylabel('Aceleracion rads/s^2')
title('Aceleracion articular Articular - Cinemática Inversa Analítica')
grid on

figure
plot(tvec, d_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(tvec, d_angular(:,2), 'g', 'LineWidth', 1.5);
plot(tvec, d_angular(:,3), 'b', 'LineWidth', 1.5);
legend('\theta_1', '\theta_2', '\theta_3')
xlabel('Tiempo (s)')
ylabel('Ángulo (rad)')
title('Trayectoria Articular - Cinemática Inversa Analítica')
grid on


muestras_2 = size(p_angular,1);
puntos_cart=zeros(muestras_2,3);

for i=1:muestras_2

    [x, y, z] = cinematica_directa(p_angular(i,1), p_angular(i,2), p_angular(i,3), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
    puntos_cart(i,1) = x; 
    puntos_cart(i,2) = y; 
    puntos_cart(i,3) = z;

end

figure
plot3(puntos_cart(:,1), puntos_cart(:,2), puntos_cart(:,3), 'b', 'LineWidth', 1.5)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Trayectoria cartesiana')
grid on
axis equal
view(45, 25)

hold on
scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 80, 'r', 'filled')

%% Parametros motores

R1  = 2.7;      % Ohms
L1  = 0.0011;   % H
Kt1 = 2.92;     % Nm/A
Kb1 = 2.865;    % V/(rad/s)
J1  = 0.0216;   % kg·m²
B1  = 0.0334;   % Nm/(rad/s)

R2  = 4;        % Ohms
L2  = 0.0016;   % H
Kt2 = 2.46;     % Nm/A
Kb2 = 2.4828;   % V/(rad/s)
J2  = 0.011;    % kg·m²
B2  = 0.1719;   % Nm/(rad/s)

%% Límites articulares (ajusta según tu robot)

theta1_lim = linspace(-2.79253,    2.79253,    50);
theta2_lim = linspace(-2.4435,    0.1838,     50);
theta3_lim = linspace(0.698132,      2.44346,    50);

%% Generar nube de puntos
n = length(theta1_lim) * length(theta2_lim) * length(theta3_lim);
workspace = zeros(n, 3);

idx = 1;
for t1 = theta1_lim
    for t2 = theta2_lim
        for t3 = theta3_lim
            [x, y, z] = cinematica_directa(t1, t2, t3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
            workspace(idx, :) = [x, y, z];
            idx = idx + 1;
        end
    end
end


%% Grafica
figure
scatter3(workspace(:,1), workspace(:,2), workspace(:,3), 1, workspace(:,3), 'filled')
colormap jet
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Espacio de trabajo del robot manipulador')
xlim([-0.5  0.5])
ylim([-0.5  0.5])
zlim([ 0.0  0.4])
grid on
axis equal
view(45, 25)

hold on
scatter3(pos_home(1), pos_home(2), pos_home(3), 100, 'k', 'filled')
scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 100, 'black', 'filled')




