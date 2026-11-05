function [x, y, z] = cinematica_directa(theta1, theta2, theta3, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)

    x = cos(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2)) - dz_tcp*sin(theta1) + dx_tcp*cos(theta2 + theta3)*cos(theta1) - dy_tcp*sin(theta2 + theta3)*cos(theta1);
    y = sin(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2)) + dz_tcp*cos(theta1) + dx_tcp*cos(theta2 + theta3)*sin(theta1) - dy_tcp*sin(theta2 + theta3)*sin(theta1);
    z = d1 - dy_tcp*cos(theta2 + theta3) - a3*sin(theta2 + theta3) - dx_tcp*sin(theta2 + theta3) - a2*sin(theta2);

end
