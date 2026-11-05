function [p, dp, ap, t_total] = QUINTIC_POLY(waypoints, tiempos, iteraciones) 

    wy = size(waypoints,2);
    
    if length(tiempos) ~= wy
        error('La matriz de tiempos debe tener la misma longitud que la matriz de waypoints.');
    end
     
    p = zeros(1,iteraciones*(wy-1));
    dp = zeros(1,iteraciones*(wy-1));
    ap = zeros(1,iteraciones*(wy-1));
    
    for j=1:wy-1
    
        idx2 = j;
        [qt, qtp, qtpp] = polinomio_quinto(waypoints(j), waypoints(j+1), tiempos(j), tiempos(j+1), iteraciones);
        inicio = (j - 1) * iteraciones + 1;
        fin = j * iteraciones;
        p(inicio:fin) = qt;
        dp(inicio:fin) = qtp;
        ap(inicio:fin) = qtpp;
    
    end
    
    t_total = linspace(tiempos(1), tiempos(end), iteraciones*(wy-1));

end


% function [p, dp, ap] = QUINTIC_POLY(waypoints, tiempos, iteraciones) 
% 
%     wy = size(waypoints,2);
% 
%     if length(tiempos) ~= wy
%         error('La matriz de tiempos debe tener la misma longitud que la matriz de waypoints.');
%     end
% 
%     V = zeros(1, wy);
%     A = zeros(1, wy);
% 
%     for i = 2:(wy-1)
%         V(i) = (waypoints(i+1) - waypoints(i-1)) / (tiempos(i+1) - tiempos(i-1));
%     end
% 
%     for i = 2:(wy-1)
%         A(i) = (V(i+1) - V(i-1)) / (tiempos(i+1) - tiempos(i-1));
%     end
% 
%     p = zeros(1,iteraciones*(wy-1));
%     dp = zeros(1,iteraciones*(wy-1));
%     ap = zeros(1,iteraciones*(wy-1));
% 
%     for j=1:wy-1
% 
%         idx2 = j;
%         [qt, qtp, qtpp] = polinomio_quinto(waypoints(j), waypoints(j+1), V(j), V(j+1), A(j), A(j+1), tiempos(j), tiempos(j+1), iteraciones);
%         inicio = (j - 1) * iteraciones + 1;
%         fin = j * iteraciones;
%         p(inicio:fin) = qt;
%         dp(inicio:fin) = qtp;
%         ap(inicio:fin) = qtpp;
% 
%     end
% 
%     t_total = linspace(tiempos(1), tiempos(end), iteraciones*(wy-1));
%     figure
%     plot(t_total, p, 'r', 'LineWidth', 1.5)
%     title('Posicion')
%     grid on
% 
%     figure
%     plot(t_total, dp, 'g', 'LineWidth', 1.5)
%     title('Velocidad')
%     grid on
% 
%     figure
%     plot(t_total, ap, 'b', 'LineWidth', 1.5)
%     title('Aceleracion')
%     grid on
% 
% end




