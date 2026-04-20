% ========================================================================
% Fun_ForwardKinematics_3R.m — Cinemática direta (3R, ferramenta fixa)
% Formato Craig: [α_{i-1}, a_{i-1}, d_i, θ_i]
% ========================================================================
function [T_all, A_all, T0_tool] = Fun_ForwardKinematics(q)
% Entradas:
%   q [3x1] = [q1; q2; q3] (rad)
% Saídas:
%   T_all {5x1} = {T0_1, T0_2, T0_3, T0_4, T0_tool}   (acumuladas)
%   A_all {5x1} = {A01, A12, A23, A34, A4tool}        (elementares)
%   T0_tool [4x4] = homogênea total até a ferramenta
% ========================================================================

    if numel(q) ~= 3
        error('Fun_ForwardKinematics_3R: vetor q deve ter 3 elementos.');
    end

    q1 = rad2deg(q(1));
    q2 = rad2deg(q(2));
    q3 = rad2deg(q(3));

    % ----------------- DH -----------------
    DH = [  0,    0.000,  0.175,  q1;     % Junta 1 (Z)
           90,    0.000,  0.000,  q2;     % Junta 2 (Y)
            0,    0.350,  0.000,  q3;     % Junta 3 (Y)
            0,    0.325,  0.000,   0;     % Elo fixo
            0,    0.150,  0.000,   0 ];   % Ferramenta fixa

    % ----------------- função homogênea de Craig (Eq. 3.6) -----------------
    A_fun = @(alpha, a, d, theta) [ ...
        cosd(theta), -sind(theta), 0, a; ...
        sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d; ...
        sind(theta)*sind(alpha), cosd(theta)*sind(alpha),  cosd(alpha),  cosd(alpha)*d; ...
        0, 0, 0, 1 ];

    % ----------------- inicialização -----------------
    n = size(DH,1);
    A_all = cell(n,1);
    T_all = cell(n,1);
    T = eye(4);

    % ----------------- cálculo -----------------
    for i = 1:n
        alpha = DH(i,1);
        a     = DH(i,2);
        d     = DH(i,3);
        theta = DH(i,4);
        A_all{i} = A_fun(alpha, a, d, theta);  % ^{i-1}T_i
        T = T * A_all{i};                      % ^0T_i
        T_all{i} = T;
    end

    % ----------------- saída final -----------------
    T0_tool = T_all{end};
end
