function [A_xz, B_xz, C_xz, D_xz, A_y, B_y, C_y, D_y, w] = Fun_HillCW(raio_orbital_target, mu_Terra, massaChaser)
% -------------------------------------------------------------------------
% HillCW.m
% 
% Matrizes LTI das equações de Hill/Clohessy–Wiltshire
%
% Estados no LVLH centrado no target:
%   In-plane (xz)   : [x; z; vx; vz]
%   Out-of-plane (y): [y; vy]
%
% Entradas (acelerações de controle):
%   u_xz = [ax; az]
%   u_y  = ay
%
% Retorna:
%   A_xz (4x4), B_xz (4x2), C_xz (4x4), D_xz (4x2)
%   A_y  (2x2), B_y  (2x1), C_y  (2x2), D_y  (2x1)
%   w = sqrt(mu/r^3) [rad/s]
%
% Equações CW (forma padrão com w):
%   x" - 2 w z'           = ax
%   y" + w^2 y            = ay
%   z" + 2 w x' - 3 w^2 z = az
% -------------------------------------------------------------------------

    % --- Constantes / argumentos
    if nargin < 2
        try
            mu_Terra = Constantes.mu_Terra; % [m^3/s^2]
        catch
            error('HillCW:ConstantesMu', 'mu_Terra não fornecido e Constantes.mu_Terra indisponível.');
        end
    end

    if nargin < 1 || isempty(raio_orbital_target)
        try
            rT = Constantes.raio_Terra;                 % [m]
            hT = Constantes.altitude_target_inicial;    % [m]
            raio_orbital_target = rT + hT;
        catch
            error('HillCW:ConstantesRaio', 'raio_orbital_target não fornecido e Constantes.{raio_Terra, altitude_target_inicial} indisponíveis.');
        end
    end

    validar_positivo(raio_orbital_target, 'raio_orbital_target [m]');
    validar_positivo(mu_Terra,           'mu_Terra [m^3/s^2]');

    % --- Movimento médio da órbita do target
    w = sqrt(mu_Terra / (raio_orbital_target^3)); % [rad/s]
    mc = Constantes.massaChaser;
    
    % --- In-plane (xz): estados [x; z; vx; vz], entradas [ax; az]
    A_xz = [ 0   0      1    0 ;
             0   0      0    1 ;
             0   0      0    2*w;
             0   3*w^2 -2*w  0 ];
    
    B_xz = [ 0     0    ;
             0     0    ;
             1/mc  0    ;
             0     1/mc ];
    
    C_xz = eye(4);
    D_xz = zeros(4,2);

    % --- Out-of-plane (y): estados [y; vy], entrada ay
    A_y = [ 0   1 ;
            w^2 0 ];
    
    B_y = [ 0 ;
            1/mc ];
    
    C_y = eye(2);
    D_y = zeros(2,1);

end

% ---------- Auxiliar ----------
function validar_positivo(valor, nome)
    if ~isscalar(valor) || ~isfinite(valor) || valor <= 0
        error('HillCW:ValorInvalido', '%s deve ser escalar, finito e positivo.', nome);
    end
end
