%% ================================================================
%  Constantes.m
% ================================================================

classdef Constantes
    properties (Constant)
        %% ===== Parâmetros físicos orbitais
        mu_Terra   = 3.986004418e14; % [m3/s2]
        raio_Terra = 6378.137e3;     % [m]
        perturbacao_J2_Terra = 1.08262668e-3;  % [-] coef de achatamento
        g0          = 9.80665;        % [m/s^2] aceleração padrão da gravidade
        massaChaser = 4.6788; % [kg]

        %% ===== Perturbações
        perturbacao_target                = 0;    % perturbação no TARGET
        ECI_erro_injecao_chaser_radial    = -500;    % perturbação no CHASER eixo x absoluto [m]
        ECI_perturbacao_chaser_alongtrack = 0;    % perturbação no CHASER eixo y absoluto [1e-4 a -6]
        
        relativo_perturbacao_chaser_radial     = 0; % perturbação LVLH x no CHASER
        relativo_perturbacao_chaser_alongtrack = 0;    % perturbação LVLH y no CHASER
        relativo_perturbacao_chaser_crosstrack = 0;    % perturbação LVLH z no CHASER
            relativo_perturbacao_chaser_vradial     = 0;    % perturbação veloc LVLH z no CHASER
            relativo_perturbacao_chaser_valongtrack = 0;    % perturbação veloc LVLH z no CHASER
            relativo_perturbacao_chaser_vcrosstrack = 0;    % perturbação veloc LVLH z no CHASER

        %% ====== Condições Orbitais Iniciais
        longitude_inercial_chaser_inicial = -7;   % [deg]
        longitude_inercial_chaser_inicial_perturbado = -90;   % [deg]

        longitude_inercial_target_inicial = 0;     % [deg]
        altitude_chaser_inicial           = 500e3; % [m
        altitude_chaser_aposTH            = 559.9e3; % [m]
        altitude_target_inicial           = 600e3; % [m]

        %% ====== Posições Relativas Desejadas
        % Alvo da transferência de Hohmann
        delta_radial_TH     = -100; % [m] chaser abaixo do alvo
        delta_alongtrack_TH = -500; % [m] chaser atrás do alvo
        delta_crosstrack_TH =    0; 
        tempo_drift         = 15*60; % [min]

        % Após equações de Hill-CW (NASA)
        delta_radial_Hill_NASA     = -2;   % [m] chaser abaixo do alvo
        delta_alongtrack_Hill_NASA = -2;   % [m] chaser atrás do alvo
        delta_crosstrack_Hill_NASA =  0;

        % Após equações de Hill-CW (Fehse)
        delta_radial_Hill     =  -1; 
        delta_alongtrack_Hill =  0.3;  
        delta_crosstrack_Hill =  0; 

        %% ====== Restrições e Zonas de Segurança
        approach_ellipsoid     = [100, 100000, 50]; % [m] dimensões da elipsoide [x, y, z]
        keepout_zone_raio      = 50;                % [m]
        distancia_extensao_MR  = 3;                 % [m] distância máx antes de abrir o MR
        diametro_porta_docking = 0.10;              % [m]

        %% ====== Parâmetros de Velocidade Relativa
        veloc_rel_max_1000m = 0.3;  % [m/s] até 1000 m
        veloc_rel_max_50m   = 0.03; % [m/s] dentro de 50 m

        %% ====== Atitudes
        %% Parâmetros das atitudes dos veículos
        attd_tgt_quaternion_inicial  = [1, 0, 0, 0]; % [q0, q1, q2, q3]
        attd_chr_inicial_euler       = [7, 8, 9];    % [deg] desvio inicial em XYZ
        attd_chr_desejada_quaternion = [1, 0, 0, 0]; % igual à do target
    
    end
end