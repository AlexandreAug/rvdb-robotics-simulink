% ========================================================================
% Fun_Jacobianas_MR_3R.m — Jacobianas linear, angular e completa (6x3)
% Coerente com Fun_ForwardKinematics_3R
% ========================================================================
function [Jv, Jw, J_total, T_all] = Fun_Jacobianas(q)
% Entradas:
%   q [3x1] -> variáveis articulares [rad]
%
% Saídas:
%   Jv [3x3]      -> Jacobiana linear na ponta (EEF)
%   Jw [3x3]      -> Jacobiana angular na ponta (EEF)
%   J_total [6x3] -> Jacobiana completa [Jv; Jw]
%   T_all {5x1}   -> Homogêneas acumuladas (base até ferramenta)
% ========================================================================

n_juntas = 3;

% -------------------------------------------------------------------------
% 1. Usa cinemática direta para obter homogêneas
% -------------------------------------------------------------------------
[T_all, ~, ~] = Fun_ForwardKinematics_3R(q);

% -------------------------------------------------------------------------
% 2. Extrai origens e eixos Z
% -------------------------------------------------------------------------
n_frames = numel(T_all);
O = zeros(3, n_frames);
Z = zeros(3, n_frames);
O(:,1) = [0;0;0];
Z(:,1) = [0;0;1];
for i = 2:n_frames
    Ti_1 = T_all{i-1};
    O(:,i) = Ti_1(1:3,4);
    Z(:,i) = Ti_1(1:3,3);
end

% -------------------------------------------------------------------------
% 3. Calcula Jacobianas para o efetuador
% -------------------------------------------------------------------------
Oe = T_all{end}(1:3,4);
Jv = zeros(3,n_juntas);
Jw = zeros(3,n_juntas);

for j = 1:n_juntas
    Jv(:,j) = cross(Z(:,j), Oe - O(:,j));
    Jw(:,j) = Z(:,j);
end

% Jacobiana completa (6x3)
J_total = [Jv; Jw];
end
