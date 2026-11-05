 clc;
clearvars;
close all;

d1 = 0.100; a1 = 0.02774; a2 = 0.210; a3 = 0.23051;
dx_tcp = 0.05034; dz_tcp = 0.0254; dy_tcp = -0.00032;

% theta1 = 0; theta2 = -pi/2; theta3 = pi/2;

px1 = 0.3085; py1 = -0.00032; pz1 = 0.2846;

% [x, y, z] = cinematica_directa_2(theta1, theta2, theta3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
% 
% fprintf("x=%.4f , y=%.5f  z=%.4f \n ", x,y,z)
% [theta1, theta2, theta3] = cinematica_inversa(x, y, z, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
% fprintf("\theta_1=%.4f , \theta_2=%.5f,  \theta_3=%.4f \n ", theta1, theta2, theta3)

pos_home = [px1; py1; pz1];

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
    [th1, th2, th3] = cinematica_inversa(p_cart(i,1), p_cart(i,2), p_cart(i,3), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
    jointAngles(i,1) = th1;
    jointAngles(i,2) = th2;
    jointAngles(i,3) = th3;
end

% plot(tvec, jointAngles(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(tvec, jointAngles(:,2), 'g', 'LineWidth', 1.5);
% plot(tvec, jointAngles(:,3), 'b', 'LineWidth', 1.5);
% legend('\theta_1', '\theta_2', '\theta_3')
% xlabel('Tiempo (s)')
% ylabel('Ángulo (rad)')
% title('Trayectoria Cartesiana - Cinemática Inversa Analítica')
% grid on

% figure
% subplot(3,1,1); plot(tvec, jointAngles(:,1), 'r', 'LineWidth', 1.5); ylabel('\theta_1 [rads]'); grid on; 
% subplot(3,1,2); plot(tvec, jointAngles(:,2), 'g', 'LineWidth', 1.5); ylabel('\theta_2 [rads]'); grid on;
% subplot(3,1,3); plot(tvec, jointAngles(:,3), 'b', 'LineWidth', 1.5); ylabel('\theta_3 [rads]'); grid on;
% xlabel('Tiempo (s)')
% title('Trayectoria Cartesiana - Cinemática Inversa Analítica')

figure
subplot(3,1,1)
plot(tvec,jointAngles(:,1),'r','LineWidth',1.5)
%title('Posición Articulación 1')
ylabel('$\theta_1$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec,jointAngles(:,2),'g','LineWidth',1.5)
%title('Posición Articulación 2')
ylabel('$\theta_2$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec,jointAngles(:,3),'b','LineWidth',1.5)
%title('Posición Articulación 3')
ylabel('$\theta_3$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on



figure
plot3(p_cart(:,1), p_cart(:,2), p_cart(:,3), 'b', 'LineWidth', 1.5)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
grid on
axis equal
view(45, 25)
hold on
scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 80, 'r', 'filled')

%% Interpolacion angular

muestras = size(wpts,2);
wpts_angular = zeros(3, muestras);

for i=1:muestras

    [th1, th2, th3] = cinematica_inversa(wpts(1,i), wpts(2,i), wpts(3,i), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
    wpts_angular(1,i) = th1; 
    wpts_angular(2,i) = th2; 
    wpts_angular(3,i) = th3; 

end

[p_angular, d_angular, a_angular] = quinticpolytraj(wpts_angular,tpts,tvec);
p_angular = p_angular';
d_angular = d_angular';
a_angular = a_angular';



% figure
% plot(tvec, p_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(tvec, p_angular(:,2), 'g', 'LineWidth', 1.5);
% plot(tvec, p_angular(:,3), 'b', 'LineWidth', 1.5);
% legend('\theta_1', '\theta_2', '\theta_3')
% xlabel('Tiempo (s)')
% ylabel('Posicion rads')
% title('Trayectoria Articular - Cinemática Inversa Analítica')
% grid on

% figure
% subplot(3,1,1); plot(tvec, p_angular(:,1), 'r', 'LineWidth', 1.5); ylabel('\theta_1 [rads]'); grid on; 
% subplot(3,1,2); plot(tvec, p_angular(:,2), 'g', 'LineWidth', 1.5); ylabel('\theta_2 [rads]'); grid on;
% subplot(3,1,3); plot(tvec, p_angular(:,3), 'b', 'LineWidth', 1.5); ylabel('\theta_3 [rads]'); grid on;
% xlabel('Tiempo (s)')
% title('Trayectoria Articular - Cinemática Inversa Analítica')
figure
subplot(3,1,1)
plot(tvec,p_angular(:,1),'r','LineWidth',1.5)
%title('Posición Articular 1')
ylabel('$\theta_1$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec,p_angular(:,2),'g','LineWidth',1.5)
%title('Posición Articular 2')
ylabel('$\theta_2$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec,p_angular(:,3),'b','LineWidth',1.5)
%title('Posición Articular 3')
ylabel('$\theta_3$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on


% figure
% plot(tvec, a_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(tvec, a_angular(:,2), 'g', 'LineWidth', 1.5);
% plot(tvec, a_angular(:,3), 'b', 'LineWidth', 1.5);
% legend('\theta_1', '\theta_2', '\theta_3')
% xlabel('Tiempo (s)')
% ylabel('Aceleracion rads/s^2')
% title('Aceleracion Articular - Cinemática Inversa Analítica')
% grid on
figure

subplot(3,1,1)
plot(tvec,a_angular(:,1),'r','LineWidth',1.5)
title('Aceleracion Articulación 1')
ylabel('$\ddot{\theta}_1$ (rad/s$^2$)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec,a_angular(:,2),'g','LineWidth',1.5)
title('Aceleracion Articulación 2')
ylabel('$\ddot{\theta}_2$ (rad/s$^2$)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec,a_angular(:,3),'b','LineWidth',1.5)
title('Aceleracion Articulación 3')
ylabel('$\ddot{\theta}_3$ (rad/s$^2$)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on


% figure
% plot(tvec, d_angular(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(tvec, d_angular(:,2), 'g', 'LineWidth', 1.5);
% plot(tvec, d_angular(:,3), 'b', 'LineWidth', 1.5);
% legend('\theta_1', '\theta_2', '\theta_3')
% xlabel('Tiempo (s)')
% ylabel('Velocidad (rads/s)')
% title('Velocidad Articular - Cinemática Inversa Analítica')
% grid on

figure
subplot(3,1,1)
plot(tvec,d_angular(:,1),'r','LineWidth',1.5)
title('Velocidad Articulación 1')
ylabel('$\dot{\theta}_1$ (rad/s)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec,d_angular(:,2),'g','LineWidth',1.5)
title('Velocidad Articulación 2')
ylabel('$\dot{\theta}_2$ (rad/s)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec,d_angular(:,3),'b','LineWidth',1.5)
title('Velocidad Articulación 3')
ylabel('$\dot{\theta}_3$ (rad/s)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on


muestras_2 = size(p_angular,1);
puntos_cart=zeros(muestras_2,3);

for i=1:muestras_2

    [x, y, z] = cinematica_directa_2(p_angular(i,1), p_angular(i,2), p_angular(i,3), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
    puntos_cart(i,1) = x; 
    puntos_cart(i,2) = y; 
    puntos_cart(i,3) = z;

end


figure
plot3(puntos_cart(:,1), puntos_cart(:,2), puntos_cart(:,3), 'b', 'LineWidth', 1.5)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
grid on
axis equal
view(45, 25)

hold on
scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 80, 'r', 'filled')

% %% Parametros motores
% 
% R1  = 2.7;      % Ohms
% L1  = 0.0011;   % H
% Kt1 = 2.92;     % Nm/A
% Kb1 = 2.865;    % V/(rad/s)
% J1  = 0.0216;   % kg·m²
% B1  = 0.0334;   % Nm/(rad/s)
% 
% R2  = 4;        % Ohms
% L2  = 0.0016;   % H
% Kt2 = 2.46;     % Nm/A
% Kb2 = 2.4828;   % V/(rad/s)
% J2  = 0.011;    % kg·m²
% B2  = 0.1719;   % Nm/(rad/s)
% 
%% Límites articulares



% theta2_lim = linspace(-2.4435, 0.1838, 50);
% theta3_lim = linspace(0.698132, 2.44346, 50);
% 
% n = length(theta2_lim)*length(theta3_lim);
% pts_rz = zeros(n,2);
% idx = 1;
% for t2 = theta2_lim
%     for t3 = theta3_lim
%         [x, y, z] = cinematica_directa(0, t2, t3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);
%         pts_rz(idx,:) = [hypot(x,y), z];
%         idx = idx + 1;
%     end
% end
% 
% figure
% plot(pts_rz(:,1), pts_rz(:,2), '.', 'MarkerSize', 4)
% xlabel('r (m)'); ylabel('Z (m)')
% sgtitle('Sección transversal del espacio de trabajo alcanzable (plano r-Z)')
% axis equal; grid on
% 
% r_vals = pts_rz(:,1);  % radios ya calculados en el plano r-Z
% 
% theta1_lim = linspace(-2.79253, 2.79253, 50);
% 
% [R, TH] = meshgrid(r_vals, theta1_lim);
% X = R .* cos(TH);
% Y = R .* sin(TH);
% 
% figure
% plot(X(:), Y(:), '.', 'MarkerSize', 10)
% axis equal; grid on
% xlabel('X (m)'); ylabel('Y (m)')
% sgtitle('Proyección del espacio de trabajo alcanzable (plano X-Y)')
% 
% % theta1_lim = linspace(-2.79253, 2.79253, 100);  % ahora solo para graficar
% % [R, TH] = meshgrid(pts_rz(:,1), theta1_lim);
% Zsurf = repmat(pts_rz(:,2)', length(theta1_lim), 1);
% % X = R.*cos(TH); Y = R.*sin(TH);
% % 
% figure
% surf(X, Y, Zsurf, 'FaceAlpha', 0.5, 'EdgeColor', 'none')
% axis equal; grid on
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z(m)')



