clc;
clearvars;
close all;


d1 = 0.100; a1 = 0.02774; a2 = 0.210; a3 = 0.23051;
dx_tcp = 0.05034; dz_tcp = 0.0254; dy_tcp = -0.00032;
th1= 0; th2 = -pi/2; th3 = pi/2;

r = (a2 * cos(th2))/2;
h = (a2 * sin(th2))/2;

y = (a1 + r)/sin(th1);
x = (a1 + r)/cos(th1);
z = d1+ h;
