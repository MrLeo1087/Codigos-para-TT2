clear; clc; close all;

d1 = 0.100;  
a1 = 0.02774;  
a2 = 0.08;  
a3 = 0.21; 


L(1) = Link('d', d1, 'a', a1, 'alpha', pi/2, 'offset', 0);
L(2) = Link('d', 0, 'a', a2, 'alpha', 0,    'offset', 0);
L(3) = Link('d', 0, 'a', a3, 'alpha', 0,    'offset', 0);

robot = SerialLink(L, 'name', 'Manipulador CINVESTAV');

q1_motor      = 0;          
q2_motor      = 0;      
q3_motor      = 0;

q_dh = [q1_motor, q2_motor, q3_motor];

disp('Matriz de Transformación Homogénea (Cinemática Directa):');
T = robot.fkine(q_dh) 

figure('Name', 'Simulación del Manipulador');
robot.plot(q_dh, 'workspace', [-0.5 0.65 -0.5 0.65 -0.25 0.55], 'scale', 0.5, 'noname');
disp('Abriendo modo Teach. Nota: Los deslizadores mueven la cadena abierta.');
robot.teach(q_dh);


