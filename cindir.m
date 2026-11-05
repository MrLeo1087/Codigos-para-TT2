function [x, y, z] = cindir(th1, th2, th3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)

    r = a2*cos(th2)-a3*cos(th3-th2);
    x = (r+a1+dx_tcp)*sin(th1)-dy_tcp*cos(th1);
    y = (r+a1+dx_tcp)*cos(th1)-dy_tcp*sin(th1);    
    z = d1 - dz_tcp - a3*sin(th3 - th2) +a2*sin(th2);
    
end
