%% ParametrosEHCW.m
clc; 

%% ==== Constantes
mu_Terra         = Constantes.mu_Terra;
raio_Terra       = Constantes.raio_Terra;
g0               = Constantes.g0;
data_inicial_UTC = "2025-01-01 00:00:00"; % data inicial

%% ==== Condições Orbitais Iniciais (antes de TH) ====
altitude_chaser = Constantes.altitude_chaser_inicial; % [m]
altitude_target = Constantes.altitude_target_inicial; % [m]
    raio_orbital_chaser = raio_Terra + altitude_chaser; % [m]
    raio_orbital_target = raio_Terra + altitude_target; % [m]
taxa_angular_target = sqrt(Constantes.mu_Terra / (raio_orbital_target^3));

% Velocidades orbitais circulares
veloc_orbital_chaser = sqrt(mu_Terra/raio_orbital_chaser); % [m/s]
veloc_orbital_target = sqrt(mu_Terra/raio_orbital_target); % [m/s]

% Longitudes inerciais iniciais
long_inercial_chaser = Constantes.longitude_inercial_chaser_inicial; % [deg]
long_inercial_target = Constantes.longitude_inercial_target_inicial; % [deg]

%% ==== Metas e Tolerâncias por Fase (ECI e LVLH) ====
%% 1) Após a Transferência de Hohmann
tolTH = 1/100; % [%] de tolerância por eixo

% Bandas posicionais (radial, along-track e opcional cross-track)
delta_radial_TH     = Constantes.delta_radial_TH;
delta_alongtrack_TH = Constantes.delta_alongtrack_TH;
delta_crosstrack_TH = Constantes.delta_crosstrack_TH;

% Faixa de velocidade relativa aceitável após TH
veloc_relativa_fechamento_TH.x = [ -0.10, +0.10 ];   % [m/s] limite radial (positivo = afastando)
veloc_relativa_fechamento_TH.y = [ -0.05, +0.05 ];   % [m/s] limite along-track (deslizamento tangencial)
veloc_relativa_fechamento_TH.z = [ -0.05, +0.05 ];   % [m/s] limite cross-track (desvio de plano)

%% 2) Aproximação Curta (Hill/CW)
tol_HCW_m = 5/100; % [%] de tolerância por eixo

delta_radial      = Constantes.delta_radial_Hill     + [-tol_HCW_m, +tol_HCW_m];
delta_alongtrack  = Constantes.delta_alongtrack_Hill + [-tol_HCW_m, +tol_HCW_m];
delta_crosstrack  = Constantes.delta_crosstrack_Hill + [-tol_HCW_m, +tol_HCW_m];

veloc_fechamento_max_Hill = 1/100;       % [m/s] taxa axial máxima (1 cm/s)
veloc_lateral_max_Hill    = 5/1000;      % [m/s] taxa lateral máxima (5 mm/s)

%% 3) Contato e Acoplamento (Docking)
vel_fechamento_axial_max = 1/10;   % [m/s] taxa máxima de fechamento axial
vel_fechamento_axial_min = 1/100;  % [m/s] taxa mínima de fechamento axial
vel_lateral_max          = 5/100;  % [m/s] taxa lateral máxima (5 cm/s)

taxas_pitch_yaw_max_graus = 0.25; % [°/s]
taxa_roll_max_graus       = 0.25; % [°/s]

desalinh_lateral_max_m       = 0.10; % [m] desalinhamento lateral máximo
desalinh_pitch_yaw_max_graus = 5.0;  % [°] desalinhamento angular máximo (pitch/yaw)
desalinh_roll_max_graus      = 5.0;  % [°] desalinhamento em rolagem máximo

%% ===== Restrições e Zonas de Segurança
Restricao.AE_dimensoes     = Constantes.approach_ellipsoid;    % [m]
Restricao.KOZ_raio         = Constantes.keepout_zone_raio;     % [m]
Restricao.dist_extensao_MR = Constantes.distancia_extensao_MR; % [m]

%% ===== Condições Iniciais de Posição e Velocidade (ECI)
% Target no eixo X (longitude 0°)
raio_target_inicial       = [raio_orbital_target; 0; 0];       % [m]
velocidade_target_inicial = [0; veloc_orbital_target; 0];      % [m/s]

% Chaser "X°" atrás
raio_target_inicial       = [0; -raio_orbital_chaser; 0];      % [m]
velocidade_target_inicial = [veloc_orbital_chaser; 0; 0];      % [m/s]

%% ===== Controle PD durante o Movimento Relativo ====
PDRel_radial_mov_Kprop   = 1;
PDRel_radial_mov_Kder    = 176;
PDRel_radial_filtro = 30;
PDRel_radial_mov_sat_sup = 1;
PDRel_radial_mov_sat_inf = -PDRel_radial_mov_sat_sup;

    PDRel_alongtrack_mov_Kprop   = 1.014;
    PDRel_alongtrack_mov_Kder    = 589;
    PDRel_alongtrack_filtro = PDRel_radial_filtro;
    PDRel_alongtrack_mov_sat_sup = PDRel_radial_mov_sat_sup;
    PDRel_alongtrack_mov_sat_inf = -PDRel_alongtrack_mov_sat_sup;

PDRel_crosstrack_mov_Kprop   = PDRel_radial_mov_Kprop;
PDRel_crosstrack_mov_Kder    = PDRel_radial_mov_Kder;
PDRel_crosstrack_filtro      = PDRel_radial_filtro;
PDRel_crosstrack_mov_sat_sup = PDRel_radial_mov_sat_sup;
PDRel_crosstrack_mov_sat_inf = -PDRel_crosstrack_mov_sat_sup;

%% ===== Guiamento, Navegação e Controle
% Controle PD para a atitude
attd_Gprop = 15;
attd_Gder = 1;
attd_filterN = 30;
attd_tau = 99;

attd_Kprop = [attd_Gprop;attd_Gprop;attd_Gprop];
attd_Kder  = [attd_Gder;attd_Gder;attd_Gder];
    PD_Attde_sup   = 5;
    PD_Attde_inf   = -PD_Attde_sup;

attd_tau_max = [attd_tau;attd_tau;attd_tau];

%% 2) Referência de atitude (alvo, quaternion [w x y z])
attd_target_quaternion = Constantes.attd_tgt_quaternion_inicial;

%% 3) Estado inicial do chaser
% attd_inicial_chaser_euler       = Constantes.attd_chr_inicial_euler;
attd_inicial_chaser_quaternion    = Fun_Euler2Quat(Constantes.attd_chr_inicial_euler);
attd_chaser_desejada_quaternion   = attd_target_quaternion;
attd_velocAngular_inicial_chaser  = zeros(3, 1);
attd_velocAngular_desejada_chaser = zeros(3, 1);

%% 4) Matriz de inércia do chaser
attd_MI_chaser_home = tensorInercia_chaser_home;

%% 5) Perturbação que o MR incide na base
perturbacao_MR_base = 0;