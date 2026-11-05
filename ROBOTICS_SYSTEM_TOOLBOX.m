% clc; clearvars; close all;
% 
% d1 = 0.100;
% a1 = 0.02774;
% a2 = 0.210;
% a3 = 0.23051;
% a4 = 0.05034;   % offset axial del TCP
% dz_tcp = 0.0254;    % offset lateral del TCP (en d del marco 4)
% d2 = 0.00032;
% 
% dhparams = [a1   -pi/2   d1      0;
%             a2    0       0      0;
%             a3    0       0      0;
%             a4   -pi/2   d2      0];   % theta del 4to se controla aparte
% 
% robot = rigidBodyTree('DataFormat', 'row');
% 
% bodies = cell(4,1);
% joints = cell(4,1);
% 
% for i = 1:4
%     bodies{i} = rigidBody(['body' num2str(i)]);
%     joints{i} = rigidBodyJoint(['jnt' num2str(i)], "revolute");
%     setFixedTransform(joints{i}, dhparams(i,:), "dh");
%     bodies{i}.Joint = joints{i};
%     if i == 1
%         addBody(robot, bodies{i}, "base");
%     else
%         addBody(robot, bodies{i}, bodies{i-1}.Name);
%     end
% end
% 
% efector = rigidBody('end_effector');
% jnt_ef = rigidBodyJoint('jnt_ef', 'fixed');
% setFixedTransform(jnt_ef, trvec2tform([0, 0, dz_tcp]));
% efector.Joint = jnt_ef;
% addBody(robot, efector, 'body4');
% 
% showdetails(robot)
% 
% expand_q = @(q) [q(1), q(2), q(3), -(q(2)+q(3))];
% 
% q_home = [0, -pi/2, pi/2];
% T_home = getTransform(robot, expand_q(q_home), 'end_effector', 'base');
% pos_home = T_home(1:3,4);
% disp('Posición home:'); disp(pos_home')
% 
% figure; show(robot, expand_q(q_home)); title('Home')
% figure; show(robot, expand_q([0 0 0])); title('Ceros')
% 
% poses = [0 -pi/2 pi/2; 0 -pi/3 pi/2; pi/4 -pi/2 pi/3];
% 
% for i = 1:size(poses,1)
%     q = poses(i,:);
%     T = getTransform(robot, expand_q(q), 'end_effector', 'base');
%     [xa,ya,za] = cinematica_directa_2(q(1),q(2),q(3), d1,a1,a2,a3, a4,d2,dz_tcp);
%     fprintf('Pose %d: rbt=[%.4f %.4f %.4f]  analitica=[%.4f %.4f %.4f]\n', ...
%             i, T(1:3,4), xa,ya,za);
% end
% 
% for i = 1:size(poses,1)
%     th = poses(i,:);
%     [x,y,z] = cinematica_directa_2(th(1),th(2),th(3), d1,a1,a2,a3, a4, d2,dz_tcp);
%     [t1,t2,t3] = cinematica_inversa(x,y,z, d1,a1,a2,a3, a4, d2,dz_tcp);
%     fprintf('Pose %d: dth1=%.2e  dth2=%.2e  dth3=%.2e\n', ...
%             i, t1, t2, t3);
% end
% 
% axis equal
% grid on 
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% title('Modelo cinemático del robot manipulador')
% view(25, 25) 
% 
% %% Etiquetas de cada marco
% hold on
% 
% bodies_list = {'base', 'body1', 'body2', 'body3', 'end_effector'};
% labels      = {'O_0', 'O_1',   'O_2',   'O_3',   'TCP'};
% 
% for i = 1:length(bodies_list)
%     T   = getTransform(robot, q_home, bodies_list{i}, 'base');
%     pos = T(1:3, 4);
%     text(pos(1)+ 0.035, pos(2)+ 0.035, pos(3) + 0.035, labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', 'HorizontalAlignment', 'center')
% end
% 
% %% Marcar articulaciones
% 
% for i = 1:3
%     T   = getTransform(robot, q_home, ['body' num2str(i)], 'base');
%     pos = T(1:3, 4);
%     plot3(pos(1), pos(2), pos(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b')
% end
% 
% %% Marcar TCP
% 
% T_tcp = getTransform(robot, q_home, 'end_effector', 'base');
% p_tcp = T_tcp(1:3, 4);
% plot3(p_tcp(1), p_tcp(2), p_tcp(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
% 
% 
% %% Generación de trayectorias (Espacio Cartesiano)

%%     POS SEGURA                RECOJE             REGRESA               DEPOSITA          REGRESA POS SEGURA
% wpts = [pos_home(1)  -0.00032   -0.00032  -0.00032  pos_home(1)  0.00032  0.00032  0.00032  pos_home(1);
%         pos_home(2)  -0.30808   -0.27494  -0.30808  pos_home(2)  0.30808  0.27494  0.30808  pos_home(2);
%         pos_home(3)   0.28460    0.01553   0.28460  pos_home(3)  0.28460  0.01553  0.28460  pos_home(3)];
% 
% tpts = [0, 3, 6, 9, 12, 15, 18, 21, 24];
% tvec = 0:0.001:24;
% 
% [p_cart, p_cart_vel, p_cart_acc, pp] = cubicpolytraj(wpts, tpts, tvec);
% cartesian_trajectory = p_cart'; % Nx3
% 
% %% Generación de cinemática inversa para la trayectoria (Espacio Articular)
% 
% gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position'});
% posTgt = constraintPositionTarget("end_effector");
% 
% posTgt.TargetPosition = [0.13995, 0.0, 0.20818];
% [q_reposo, ~] = gik(q_home, posTgt);
% 
% nSamples = length(tvec); % 5001 muestras
% jointAngles = zeros(nSamples, 3); 
% initguess = q_home;
% T_check = getTransform(robot, q_home, 'end_effector', 'base');
% pos_check = T_check(1:3, 4);
% 
% 
% for i = 1:nSamples
%     posTgt.TargetPosition = cartesian_trajectory(i, :);
%     [config, solInfo] = gik(initguess, posTgt);
%     jointAngles(i, :) = config;
%     initguess = config; % Actualizamos el guess inicial para convergencia rápida
% end
% 
% figure
% plot(tvec(1:end-1), diff(jointAngles(:,2)), 'g')
% hold on
% plot(tvec(1:end-1), diff(jointAngles(:,3)), 'b')
% title('Derivada de angulos - detectar saltos')
% grid on
% 
% %% Gráficos
% figure
% plot(tvec, jointAngles(:,1), 'r', 'LineWidth', 1.5); hold on;
% plot(tvec, jointAngles(:,2), 'g', 'LineWidth', 1.5);
% plot(tvec, jointAngles(:,3), 'b', 'LineWidth', 1.5);
% legend('\theta_1', '\theta_2', '\theta_3')
% xlabel('Tiempo (s)')
% ylabel('Ángulo (rad)')
% title('Trayectoria Articular')
% grid on
% 
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


clc;
clearvars;
close all;

%% Definición de medidas físicas
d1 = 0.100;  
a1 = 0.02774;  
a2 = 0.210;  
a3 = 0.23051; 


%% Definimos parámetros Denavit-Hartenberg

dhparams = [a1      -pi/2   d1      0;
            a2      0       0       0;
            a3      0       0       0];

robot = rigidBodyTree('DataFormat', 'row');
bodies = cell(3,1);
joints = cell(3,1);

%% Creación de eslabones

for i = 1:3

    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], "revolute");
    setFixedTransform(joints{i}, dhparams(i,:), "dh");
    bodies{i}.Joint = joints{i};
    if i == 1
        addBody(robot, bodies{i}, "base")
    else
        addBody(robot, bodies{i}, bodies{i-1}.Name)
    end

end

efector = rigidBody('end_effector');
jnt_efector = rigidBodyJoint('jnt_efector', 'fixed');
dx_tcp = 0.05034;
dy_tcp = 0.0254;
dz_tcp = -0.00032;
T_herramienta = trvec2tform([dx_tcp, dy_tcp, dz_tcp]);
setFixedTransform(jnt_efector, T_herramienta);
efector.Joint = jnt_efector;
addBody(robot, efector, 'body3');

% showdetails(robot) 

%% Definición de home y Cinemática Directa

q_home = [0, -pi/2, pi/2];
T_home = getTransform(robot, q_home, 'end_effector', 'base');
pos_home = T_home(1:3, 4);

%% Generación de trayectorias (Espacio Cartesiano)

%       POS SEGURA                RECOJE             REGRESA              DEPOSITA        REGRESA POS SEGURA
wpts = [pos_home(1)  -0.00032   -0.00032  -0.00032  pos_home(1)  0.00032  0.00032  0.00032  pos_home(1);
        pos_home(2)  -0.30808   -0.27494  -0.30808  pos_home(2)  0.30808  0.27494  0.30808  pos_home(2);
        pos_home(3)   0.28460    0.01553   0.28460  pos_home(3)  0.28460  0.01553  0.28460  pos_home(3)];

tpts = [0, 3, 6, 9, 12, 15, 18, 21, 24];
tvec = 0:0.001:24;



% [p_cart, p_cart_vel, p_cart_acc, pp] = cubicpolytraj(wpts, tpts, tvec);
% cartesian_trajectory = p_cart'; % Nx3
% 
% %% Generación de cinemática inversa para la trayectoria (Espacio Articular)
% 
% gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position'});
% posTgt = constraintPositionTarget("end_effector");
% 
% nSamples = length(tvec); % 5001 muestras
% jointAngles = zeros(nSamples, 3); 
% initguess = q_home;
% T_check = getTransform(robot, q_home, 'end_effector', 'base');
% pos_check = T_check(1:3, 4);
% 
% for i = 1:nSamples
%     posTgt.TargetPosition = cartesian_trajectory(i, :);
%     [config, solInfo] = gik(initguess, posTgt);
%     jointAngles(i, :) = config;
%     initguess = config; 
% end

num_wpts = size(wpts, 2);
q_wpts = zeros(3, num_wpts); % Matriz para almacenar los waypoints articulares (3x9)

gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position'});
posTgt = constraintPositionTarget("end_effector");
initguess = q_home;

for i = 1:num_wpts
    posTgt.TargetPosition = wpts(:, i)'; % Asignar la posición cartesiana (1x3)
    [config, ~] = gik(initguess, posTgt);
    q_wpts(:, i) = config'; % Guardar configuración articular como columna
    initguess = config; 
end

%% 2. Generación de Trayectoria Interpolada (Espacio Articular)
% Interpolar los waypoints articulares con polinomios de quinto grado
[q_traj, q_traj_vel, q_traj_acc] = quinticpolytraj(q_wpts, tpts, tvec);

% Transponer para que el resultado sea Nx3 y compatible con tus gráficas
jointAngles = q_traj';

%% Gráficos


figure
sgtitle('Robotics System Toolbox')
subplot(3,1,1)
plot(tvec, jointAngles(:,1),'r','LineWidth',1.5)
ylabel('$\theta_1$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,2)
plot(tvec, jointAngles(:,2),'g','LineWidth',1.5)
ylabel('$\theta_2$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on
subplot(3,1,3)
plot(tvec, jointAngles(:,3),'b','LineWidth',1.5)
ylabel('$\theta_3$ (rad)','Interpreter','latex')
xlabel('Tiempo (s)')
grid on

figure
show(robot, q_home, 'Frames', 'on');
title('Robot en Configuración Segura')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
view(45, 30) 

% Ajuste manual de los límites de los ejes (Zoom)
xlim([-0.2 0.4])
ylim([-0.2 0.2])
zlim([-0.1 0.5])