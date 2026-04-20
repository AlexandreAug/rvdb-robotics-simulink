% ========================================================================
% Fun_JointTrajectory.m
% ========================================================================
% Gera trajetória desejada qd(t), dq_d(t), ddq_d(t) para cada junta (4R1P)
% Pode usar perfil senoidal ou polinomial (5ª ordem)
% ========================================================================

function [qd, dq_d, ddq_d] = Fun_JointTrajectory(t, tf, q0, qf, tipo)
    % ------------------------------------------------------------
    % Entradas:
    % t   -> tempo atual
    % tf  -> tempo total da trajetória
    % q0  -> vetor de posições iniciais [5x1]
    % qf  -> vetor de posições finais [5x1]
    % tipo -> 'senoidal' ou 'polinomial'
    % ------------------------------------------------------------

    n = length(q0);
    qd    = zeros(n,1);
    dq_d  = zeros(n,1);
    ddq_d = zeros(n,1);

    s = min(max(t / tf, 0), 1); % normaliza tempo (0–1)

    for i = 1:n
        if strcmpi(tipo, 'senoidal')
            qd(i)    = q0(i) + (qf(i)-q0(i)) * (0.5 - 0.5*cos(pi*s));
            dq_d(i)  = (qf(i)-q0(i)) * (0.5*pi/tf) * sin(pi*s);
            ddq_d(i) = (qf(i)-q0(i)) * (0.5*pi^2/tf^2) * cos(pi*s);
        else
            % Polinômio de 5ª ordem
            a0 = q0(i);
            a1 = 0;
            a2 = 0;
            a3 = 10*(qf(i)-q0(i));
            a4 = -15*(qf(i)-q0(i));
            a5 = 6*(qf(i)-q0(i));
            qd(i)    = a0 + a3*s^3 + a4*s^4 + a5*s^5;
            dq_d(i)  = (3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4) / tf;
            ddq_d(i) = (6*a3*s + 12*a4*s^2 + 20*a5*s^3) / tf^2;
        end
    end
end
