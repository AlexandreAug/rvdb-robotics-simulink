% ========================================================================
% Fun_ForwardKinematics.m  — 4R1P (DH padrão)
% ========================================================================

function T_all = Fun_ForwardKinematics(q, L)

    % --- sanity check ---
    if numel(q) ~= 5 || numel(L) ~= 4
        error('Fun_ForwardKinematics: tamanhos inválidos (q=5, L=4).');
    end
    if any(isnan(q))
        error('Fun_ForwardKinematics: vetor q contém NaN');
    end

    % --- juntas ---
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); d5 = q(5);
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);

    % --- limita junta prismática (ajuste se necessário) ---
    d5 = max(0.00, min(0.15, d5));  % 0–15 cm

    % ----------------- DH (theta, d, a, alpha) -----------------
    % J1: R com deslocamento ao longo z (altura L1)
    % Demais: elos no plano com a = Lk, d = 0; J5 prismática ao longo z
    DH = [ q1,  L1,  0,   pi/2;   % J1
           q2,  0,   L2,  0;      % J2
           q3,  0,   L3,  0;      % J3
           q4,  0,   L4,  0;      % J4
           0,   d5,  0,   0];     % J5 (P)

    % ----------------- acumula transformações -------------------
    T_all = cell(5,1);
    T = eye(4);
    for i = 1:5
        th = DH(i,1); d = DH(i,2); a = DH(i,3); al = DH(i,4);
        A = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
             sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
             0        sin(al)          cos(al)          d;
             0        0                0                1];
        T = T * A;
        T_all{i} = T;
    end

end
