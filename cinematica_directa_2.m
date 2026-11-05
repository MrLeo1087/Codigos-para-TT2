function [x, y, z] = cinematica_directa_2(th1, th2, th3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)

    x = a1*cos(th1) + dx_tcp*cos(th1) + dy_tcp*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
    y = a1*sin(th1) - dy_tcp*cos(th1) + dx_tcp*sin(th1) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);    
    z = d1 - dz_tcp - a3*sin(th2 + th3) - a2*sin(th2);
    
end