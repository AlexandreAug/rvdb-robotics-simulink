% ========================================================================
% Fun_Jacobianos_Acoplados.m — versão compatível com manipulador 3R
% ========================================================================
% Retorna:
%   Jb_all — {1×n} Jacobianos da base (6×6)
%   Jm_all — {1×n} Jacobianos das juntas (6×n)
%
% Compatível com Fun_ForwardKinematics_3R e Fun_Jacobianas_MR_3R
% ========================================================================

function [Jb_all, Jm_all] = Fun_Jacobianos_Acoplados(q)

    % ---------------------------
    % 1. Número de juntas
    % ---------------------------
    n = length(q);
    if n ~= 3
        error('Fun_Jacobianos_Acoplados: esperado manipulador 3R (n=3).');
    end

    % ---------------------------
    % 2. Cinemática direta (Craig)
    % ---------------------------
    [T_all, ~, T_tool] = Fun_ForwardKinematics_3R(q);

    % ---------------------------
    % 3. Jacobianas totais (para o efetuador)
    % ---------------------------
    [Jv, Jw, J_total, ~] = Fun_Jacobianas_MR_3R(q);

    % ---------------------------
    % 4. Inicializa células
    % ---------------------------
    Jb_all = cell(1,n);
    Jm_all = cell(1,n);

    % ---------------------------
    % 5. Preenche cada elo
    % ---------------------------
    for i = 1:n
        % Jacobiano da base (identidade — base rígida)
        Jb_all{i} = eye(6);

        % Jacobiano parcial das juntas até a i-ésima
        Jm_partial = J_total(:,1:i);

        % Preenche com zeros à direita para manter dimensão 6×n
        Jm_all{i} = [Jm_partial, zeros(6, n - i)];
    end
end
