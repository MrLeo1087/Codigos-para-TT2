function [theta1, theta2, theta3] = cinematica_inversa(x, y, z, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)
    %% Calculo para theta 1

    theta1 = atan2(y, x) - atan2(-dy_tcp, sqrt(x^2 + y^2 - dy_tcp^2));

    %% Calculo para theta 3

    r = x*cos(theta1) + y*sin(theta1) - a1 - dx_tcp;
    h = d1 - z - dz_tcp;
    cos_theta3 = (r^2 + h^2 - a2^2 - a3^2) / (2*a2*a3);
    theta3 = atan2( sqrt(1 - cos_theta3^2), cos_theta3);

    %% Calculo para theta 2

    k1_up = a2 + a3*cos(theta3);
    k2_up = a3*sin(theta3);
    theta2 = atan2(h, r) - atan2(k2_up, k1_up);

end

% function [theta1, theta2, theta3] = cinematica_inversa(x, y, z, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)
% 
%     % Estimación inicial de theta1 (sin corrección de offset lateral)
%     theta1 = atan2(y, x);
% 
%     tol = 1e-12;
%     max_iter = 20;
% 
%     for iter = 1:max_iter
%         theta1_ant = theta1;
% 
%         %% Plano del brazo con el theta1 actual
%         r = x*cos(theta1) + y*sin(theta1) - a1 - dx_tcp;
%         h = d1 - z - dz_tcp;
% 
%         %% theta3 (codo arriba)
%         cos_theta3 = (r^2 + h^2 - a2^2 - a3^2) / (2*a2*a3);
%         theta3 = atan2(sqrt(1 - cos_theta3^2), cos_theta3);
% 
%         %% theta2
%         k1 = a2 + a3*cos(theta3);
%         k2 = a3*sin(theta3);
%         theta2 = atan2(h, r) - atan2(k2, k1);
% 
%         %% Recalcular theta1 con el alcance radial real R
%         R = a1 + dx_tcp + a2*cos(theta2) + a3*cos(theta2 + theta3);
%         theta1 = atan2(y, x) - atan2(dy_tcp, R);
% 
%         %% Criterio de convergencia
%         if abs(theta1 - theta1_ant) < tol
%             break;
%         end
%     end
% end


% function [theta1, theta2, theta3] = cinematica_inversa(x, y, z, d1, a1, a2, a3, dx_tcp, dy_tcp, dz_tcp)
%     %% theta1 — con corrección por offset lateral dy
%     theta1 = atan2(y, x) - atan2(dy_tcp, sqrt(x^2 + y^2 - dy_tcp^2));
% 
%     %% Reducción al plano del brazo (hasta O3)
%     r = x*cos(theta1) + y*sin(theta1) - a1 - dx_tcp;
%     h = d1 - z - dz_tcp;
% 
%     %% theta3 — ley de cosenos, codo arriba
%     cos_theta3 = (r^2 + h^2 - a2^2 - a3^2) / (2*a2*a3);
%     theta3 = atan2(sqrt(1 - cos_theta3^2), cos_theta3);
% 
%     %% theta2
%     k1 = a2 + a3*cos(theta3);
%     k2 = a3*sin(theta3);
%     theta2 = atan2(h, r) - atan2(k2, k1);
% end


% function [theta1, theta2, theta3] = cinematica_inversa(x, y, z, d1, a1, a2, a3, dx_tcp, dy_tcp)
%     %% theta1
%     theta1 = atan2(y, x);
% 
%     %% Reduccion al plano 2D
%     r = x*cos(theta1) + y*sin(theta1) - a1;
%     h = d1 - z;
% 
%     %% Iteracion para desacople exacto del TCP
%     phi = 0; 
%     tol = 1e-10;
%     max_iter = 100;
% 
%     for iter = 1:max_iter
%         % Punto de muneca descontando TCP rotado
%         rw = r - dx_tcp*cos(phi) + dy_tcp*sin(phi);
%         hw = h - dx_tcp*sin(phi) - dy_tcp*cos(phi);
% 
%         cos_t3 = (rw^2 + hw^2 - a2^2 - a3^2) / (2*a2*a3);
% 
%         if abs(cos_t3) > 1
%             error('Posición fuera del espacio de trabajo');
%         end
% 
%         theta3 = atan2(sqrt(1 - cos_t3^2), cos_t3);
%         k1 = a2 + a3*cos(theta3);
%         k2 = a3*sin(theta3);
%         theta2 = atan2(hw, rw) - atan2(k2, k1);
% 
%         phi_new = theta2 + theta3;
%         phi = phi_new;
% 
%     end
% 
% end