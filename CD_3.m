clc;
clearvars;
close all;

d1 = 0.100; a1 = 0.02774; a2 = 0.210; a3 = 0.23051;
dx_tcp = 0.05034; dz_tcp = 0.0254; dy_tcp = -0.00032;
px1 = 0.3085; py1 = -0.00032; pz1 = 0.2846;
pos_home = [px1; py1; pz1];
iteraciones = 10000;

wpts = [pos_home(1)  -0.00032   -0.00032  -0.00032  pos_home(1)  0.00032  0.00032  0.00032  pos_home(1);  %theta1 
        pos_home(2)  -0.30808   -0.27494  -0.30808  pos_home(2)  0.30808  0.27494  0.30808  pos_home(2);  %theta2
        pos_home(3)   0.28460    0.01553   0.28460  pos_home(3)  0.28460  0.01553  0.28460  pos_home(3)]; %theta3

tpts = [0, 3, 6, 9, 12, 15, 18, 21, 24];
tvec = 0:0.001:24;

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

[p1, dp1, ap1, t_total] = QUINTIC_POLY(wpts_angular(1,:), tpts, iteraciones);
[p2, dp2, ap2, ~] = QUINTIC_POLY(wpts_angular(2,:), tpts, iteraciones);
[p3, dp3, ap3, ~] = QUINTIC_POLY(wpts_angular(3,:), tpts, iteraciones);

theta3mA =  - p2 +  pi - p3;
figure 
plot(t_total, theta3mA, 'r')
sgtitle('Recorrido motor 3 funcion propia')


%% Grafica de posicion angular 

% Mi Funcion

figure
sgtitle('Funcion propia')

subplot(3,1,1)
plot(t_total, p1,'r','LineWidth',1.5)
ylabel('$\theta_1$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(t_total, p2,'g','LineWidth',1.5)
ylabel('$\theta_2$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(t_total, p3,'b','LineWidth',1.5)
ylabel('$\theta_3$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on

% Funcion MATLAB

figure
sgtitle('quinticpolytraj() de MATLAB')
subplot(3,1,1)
plot(tvec,p_angular(:,1),'r','LineWidth',1.5)
ylabel('$\theta_1$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec,p_angular(:,2),'g','LineWidth',1.5)
ylabel('$\theta_2$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec,p_angular(:,3),'b','LineWidth',1.5)
ylabel('$\theta_3$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on

