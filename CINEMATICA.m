clc;
clearvars;
close all;

d1 = 0.100; a1 = 0.02774; a2 = 0.210; a3 = 0.23051; a5 = 0.080;
dx_tcp = 0.05034; dz_tcp = 0.0254; dy_tcp = -0.00032;
x = 0.3085; y = -0.00032; z = 0.2846;
pos_home = [x; y; z];

theta1 = atan2(y, x) - atan2(dy_tcp, sqrt(x^2 + y^2 - dy_tcp^2));

%% Calculo para theta 3

r = x*cos(theta1) + y*sin(theta1) - a1 - dx_tcp;
h = -z - dz_tcp + d1;

s = sqrt(r^2+h^2);
cos_theta3 = (r^2 + h^2 - a2^2 - a3^2) / (2*a2*a3);
theta3 = atan2( sqrt(1 - cos_theta3^2), cos_theta3);


%% Calculo para theta 2

cosbeta = (a2^2 + s^2-a3^2)/(2*a2*s);
theta2 = atan2(h, r) - atan2 (sqrt(1-cosbeta^2), cosbeta);

disp (theta1)
disp (theta2)
disp (theta3)

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
    wpts_angular(2,i) = th2; 
    wpts_angular(3,i) = th3; 

end

[p_angular, d_angular, a_angular] = quinticpolytraj(wpts_angular,tpts,tvec);
p_angular = p_angular';
d_angular = d_angular';
a_angular = a_angular';

samples = size(p_angular,1);
q3 = zeros(samples,1);
for i=1:samples
    
    c = sqrt(a2^2 + a5^2 -2*a5*a2*cos(p_angular(i,3)));
    phi = acos((a2^2 + c^2 - a5^2)/(2*a2*c));
    omega = acos((c^2 + a5^2 - a2^2)/(2*a5*c));
    q3(i,1) = phi + omega + p_angular(i,2);

end

[x, y, z] = cinematica_directa_2(deg2rad(0), deg2rad(-90), deg2rad(22), d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp);

% disp(x)
% disp(y)
% disp(z)
