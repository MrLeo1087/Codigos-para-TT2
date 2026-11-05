function [qt, qtp, qtpp] = polinomio_quinto(qi, qf, ti, tf, iteraciones)
    
    ai=0; af=0; vi=0; vf=0;
    Mq = [qi; vi; ai; qf; vf; af];
    
    A_mat = [1 ti ti^2   ti^3    ti^4    ti^5;
             0 1  2*ti   3*ti^2  4*ti^3  5*ti^4;
             0 0  2      6*ti    12*ti^2 20*ti^3;
             1 tf tf^2   tf^3    tf^4    tf^5;
             0 1  2*tf   3*tf^2  4*tf^3  5*tf^4;
             0 0  2      6*tf    12*tf^2 20*tf^3];
         
    c = A_mat \ Mq; 
    
    t = linspace(ti, tf, iteraciones);
    qt = c(1) + c(2)*t + c(3)*t.^2 + c(4)*t.^3 + c(5)*t.^4 + c(6)*t.^5;
    qtp = c(2) + 2*c(3)*t + 3*c(4)*t.^2 + 4*c(5)*t.^3 + 5*c(6)*t.^4;
    qtpp = 2*c(3) + 6*c(4)*t + 12*c(5)*t.^2 + 20*c(6)*t.^3;

end

% function [qt, qtp, qtpp] = polinomio_quinto(qi, qf, vi, vf, ai, af, ti, tf, iteraciones)
% 
%     Mq = [qi; vi; ai; qf; vf; af];
% 
%     A_mat = [1 ti ti^2   ti^3    ti^4    ti^5;
%              0 1  2*ti   3*ti^2  4*ti^3  5*ti^4;
%              0 0  2      6*ti    12*ti^2 20*ti^3;
%              1 tf tf^2   tf^3    tf^4    tf^5;
%              0 1  2*tf   3*tf^2  4*tf^3  5*tf^4;
%              0 0  2      6*tf    12*tf^2 20*tf^3];
% 
%     c = A_mat \ Mq; % Resolvemos el sistema
% 
%     t = linspace(ti, tf, iteraciones);
%     qt = c(1) + c(2)*t + c(3)*t.^2 + c(4)*t.^3 + c(5)*t.^4 + c(6)*t.^5;
%     qtp = c(2) + 2*c(3)*t + 3*c(4)*t.^2 + 4*c(5)*t.^3 + 5*c(6)*t.^4;
%     qtpp = 2*c(3) + 6*c(4)*t + 12*c(5)*t.^2 + 20*c(6)*t.^3;
% 
% end

