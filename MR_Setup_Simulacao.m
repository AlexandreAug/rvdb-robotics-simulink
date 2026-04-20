%% MR_Setup_Simulacao.m

clear;
clc

%% 1. Caminhos e parâmetros principais
addpath(pwd);
addpath('..');
Setup_Orbita_Atitude;

Const = Constantes;
massa_baseGeral = double(massa_baseGeral);

load('trajetoria_Hill.mat');
    pos_input_LVLH    = [t_LVLH pos_LVLH];
    veloc_input_LVLH  = [t_LVLH vel_LVLH];
        pos_input_ECI     = [t_ECI pos_ECI];
        veloc_input_ECI   = [t_ECI veloc_ECI];

%% 2. Tempo de simulação
t0          = 0;
minutos     = 15;
tempo_final = minutos * 60; % [s]
dt          = 0.01;
tspan       = t0:dt:tempo_final;

%% 3. Parâmetros orbitais (Hill frame)
mu_Terra   = Const.mu_Terra;
raio_Terra = Const.raio_Terra;
alt_target = Const.altitude_target_inicial;
alt_chaser = Const.altitude_chaser_inicial;

R_target   = raio_Terra + alt_target;
omega_orb  = sqrt(mu_Terra / R_target^3);   % [rad/s]

delta_R0 = [Const.delta_radial_Hill; ...
            Const.delta_alongtrack_Hill; ...
            Const.delta_crosstrack_Hill];
delta_V0 = [0; 0; 0];

%% 4. Estrutura de estados (parametrização física)
estados = struct();

estados.tensorInercia_base = tensorInercia_baseGeral;
estados.massa_base         = massa_baseGeral;
estados.tensorInercia_elos = cat(3, tensorInercia_elo1, tensorInercia_elo2, tensorInercia_elo3, tensorInercia_ferramenta);
estados.massas_elos        = [massa_elo1, massa_elo2, massa_elo3, massa_ferramenta];
estados.comprimento_elos   = comprimento_elos;

estados.mu_Terra   = mu_Terra;
estados.raio_Terra = raio_Terra;
estados.omega_orb  = omega_orb;
estados.alt_target = alt_target;
estados.alt_chaser = alt_chaser;
estados.forca_disturbio = [0; 0; 0];
estados.r_CoM = COM_baseGeral;

%% 5. Tabela DH (em metros) — manipulador 3R1P
% DH = [ 0,    0.000,  0.175,  q1;     % Junta 1 (Z)
%       90,    0.000,  0.000,  q2;     % Junta 2 (Y)
%        0,    0.350,  0.000,  q3;     % Junta 3 (Y)
%        0,    0.325,  0.000,   0;     % Elo fixo
%        0,    0.150,  0.000,   0];    % Ferramenta fixa

%% 6. Saturação das juntas (Dinâmica de Lagrange)
sat_acel_base  = 2/1;
sat_veloc_base = 2/1;

sat_acel_juntas  = 2/1;
sat_veloc_juntas = 2/1;
sat_pos_juntas   = 2 * pi;

dt_bloco = 1/50;
dt_ik = 0.05;

%% 7. Controle PD — base (Euler e Lagrange)

ControlePD_tau_Euler    = 50;
ControlePD_tau_Lagrange = ControlePD_tau_Euler * 1;
PD_tau_Euler = [...
    ControlePD_tau_Euler; ...
    ControlePD_tau_Euler; ...
    ControlePD_tau_Euler];
PD_tau_Lagrange = [...
    ControlePD_tau_Lagrange;
    ControlePD_tau_Lagrange;
    ControlePD_tau_Lagrange];

Euler_Kprop = 10;
Euler_Kder  = Euler_Kprop/2;
    Euler_PD_Kprop = [Euler_Kprop; Euler_Kprop; Euler_Kprop];
    Euler_PD_Kder  = [Euler_Kder; Euler_Kder; Euler_Kder];

Lagrange_Kprop = Euler_Kprop*3;
Lagrange_Kder  = Lagrange_Kprop / 2;
Lagrange_PD_Kprop = [Lagrange_Kprop; Lagrange_Kprop; Lagrange_Kprop];
Lagrange_PD_Kder  = [Lagrange_Kder; Lagrange_Kder; Lagrange_Kder];

%% 8. Controle PD — manipulador 3R
MR_Kprop  = 1;
MR_Kder   = 1;
MR_sat    = 2;
MR_tau    = 1;
% MR_amortecimento = 1/1;
KFF = [0; 0; 0];
Kp_tarefa = 3/1;
Kp_angulo = 0;
modo_op_radial = Constantes.delta_radial_Hill * 1.2;
% disp(modo_op_radial);
modo_op_tempo = 15;
    Kp_pos = 1; % translação
    Kp_ori = 0.5; % alinhamento
    limite_veloc_junta = 0.1;
    lambda = 2;
ControlePD_MR_Kprop = [3; 3; 3];
% ControlePD_MR_Kder  = [20; 20; 20];
ControlePD_MR_Kder  = ControlePD_MR_Kprop/2;
ControlePD_MR_tau_max = [MR_sat; MR_sat; MR_sat];

twist_k_ang = 3/1;
twist_k_lat = 2/1;
twist_k_ax  = 2/1;
twist_eps_lat   = 1/10;
twist_eps_graus = 1/5;

twist_sat_veloc_ref = 1;

%% 9. Condições iniciais (pose neutra)
base_pos_inicial           = [0;0;0];
base_veloc_inicial         = [0;0;0];
base_quaternion_inicial    = attd_inicial_chaser_quaternion;
base_veloc_angular_inicial = zeros(3,1);

juntas_pos_inicial   = zeros(3,1);
juntas_veloc_inicial = zeros(3,1);
    juntas_pos_desejada  = [0; 0; 0]; 
    juntas_veloc_desejada = [0.01; 0.01; 0.01];
juntas_limite_veloc = [0.1; 0.1; 0.1];


% Estado completo
x0 = [ base_pos_inicial;
       base_veloc_inicial;
       base_quaternion_inicial;
       base_veloc_angular_inicial;
       juntas_pos_inicial;
       juntas_veloc_inicial ];

%% 10. Vetores numéricos auxiliares
estados_vetor = double([
    massa_baseGeral;
    tensorInercia_baseGeral(:);
    massa_elo1; massa_elo2; massa_elo3; massa_ferramenta;
    comprimento_elos(:)
]);

params_ts.time = 0;
params_ts.signals.values = estados_vetor(:)';
params_ts.signals.dimensions = numel(estados_vetor);

params_flat = double([
    estados.massa_base;
    estados.tensorInercia_base(:);
    estados.massas_elos(:);
    estados.comprimento_elos(:)
]);

massas_elos = [massa_elo1; massa_elo2; massa_elo3; massa_ferramenta];

%% 11. Exportação para o workspace
assignin('base','x0',x0);
assignin('base','tspan',tspan);
assignin('base','tempo_final',tempo_final);
assignin('base','delta_R0',delta_R0);
assignin('base','delta_V0',delta_V0);
assignin('base','omega_orb',omega_orb);
assignin('base','massas_elos',massas_elos);
assignin('base','estados',estados);
assignin('base','params_flat',params_flat);
% assignin('base','DH_pose_home',DH);

% =========================================================================
% RELATÓRIO (Command Window) — Conferência de parâmetros para Apêndices
% (colar no final do MR_Setup_Simulacao.m)
% =========================================================================

% fprintf('\n');
% fprintf('=====================================================================\n');
% fprintf('RELATÓRIO DE CONFERÊNCIA — MR_Setup_Simulacao (parâmetros exportados)\n');
% fprintf('=====================================================================\n');
% 
% % -----------------------------
% % A) Tempo de simulação
% % -----------------------------
% fprintf('\n[A] Tempo de simulação\n');
% fprintf('  t0             = %.6f  [s]\n', t0);
% fprintf('  minutos        = %.6f  [min]\n', minutos);
% fprintf('  tempo_final    = %.6f  [s]\n', tempo_final);
% fprintf('  dt             = %.6f  [s]\n', dt);
% fprintf('  N amostras     = %d\n', numel(tspan));
% 
% % -----------------------------
% % B) Órbita (Hill frame)
% % -----------------------------
% fprintf('\n[B] Parâmetros orbitais (Hill frame)\n');
% fprintf('  mu_Terra       = %.6e  [m^3/s^2]\n', mu_Terra);
% fprintf('  raio_Terra     = %.6e  [m]\n', raio_Terra);
% fprintf('  alt_target     = %.6f  [m]\n', alt_target);
% fprintf('  alt_chaser     = %.6f  [m]\n', alt_chaser);
% fprintf('  R_target       = %.6e  [m]\n', R_target);
% fprintf('  omega_orb      = %.6e  [rad/s]\n', omega_orb);
% fprintf('  delta_R0       = [%.6f; %.6f; %.6f]  [m]\n', delta_R0(1), delta_R0(2), delta_R0(3));
% fprintf('  delta_V0       = [%.6f; %.6f; %.6f]  [m/s]\n', delta_V0(1), delta_V0(2), delta_V0(3));
% 
% % -----------------------------
% % C) Parâmetros físicos (massa/inércia)
% % -----------------------------
% fprintf('\n[C] Parâmetros físicos (massa e inércia)\n');
% fprintf('  massa_base     = %.6f  [kg]\n', estados.massa_base);
% 
% Ibase = estados.tensorInercia_base;
% fprintf('  Inércia da base (no referencial do corpo) [kg*m^2]\n');
% fprintf('    Ixx=%.6e  Iyy=%.6e  Izz=%.6e\n', Ibase(1,1), Ibase(2,2), Ibase(3,3));
% fprintf('    Ixy=%.6e  Ixz=%.6e  Iyz=%.6e\n', Ibase(1,2), Ibase(1,3), Ibase(2,3));
% 
% fprintf('  r_CoM (base)   = [%.6f; %.6f; %.6f]  [m]\n', estados.r_CoM(1), estados.r_CoM(2), estados.r_CoM(3));
% 
% fprintf('  Massas dos elos+ferramenta [kg]\n');
% fprintf('    [m1; m2; m3; mf] = [%.6f; %.6f; %.6f; %.6f]\n', ...
%     estados.massas_elos(1), estados.massas_elos(2), estados.massas_elos(3), estados.massas_elos(4));
% 
% fprintf('  Comprimentos dos elos [m]\n');
% fprintf('    comprimento_elos = [%.6f; %.6f; %.6f]\n', ...
%     estados.comprimento_elos(1), estados.comprimento_elos(2), estados.comprimento_elos(3));
% 
% % Inércias por elo (diagonal + termos cruzados)
% Ielos = estados.tensorInercia_elos;
% nomeElo = {'Elo 1','Elo 2','Elo 3','Ferramenta'};
% fprintf('  Inércias dos elos/ferramenta (diagonais e cruzados) [kg*m^2]\n');
% for k = 1:size(Ielos,3)
%     Ik = Ielos(:,:,k);
%     fprintf('    %s:\n', nomeElo{k});
%     fprintf('      Ixx=%.6e  Iyy=%.6e  Izz=%.6e\n', Ik(1,1), Ik(2,2), Ik(3,3));
%     fprintf('      Ixy=%.6e  Ixz=%.6e  Iyz=%.6e\n', Ik(1,2), Ik(1,3), Ik(2,3));
% end
% 
% % -----------------------------
% % D) Saturações e discretizações auxiliares
% % -----------------------------
% fprintf('\n[D] Saturações e discretizações auxiliares\n');
% fprintf('  sat_acel_base       = %.6f\n', sat_acel_base);
% fprintf('  sat_veloc_base      = %.6f\n', sat_veloc_base);
% fprintf('  sat_acel_juntas     = %.6f\n', sat_acel_juntas);
% fprintf('  sat_veloc_juntas    = %.6f\n', sat_veloc_juntas);
% fprintf('  sat_pos_juntas      = %.6f  [rad]\n', sat_pos_juntas);
% fprintf('  dt_bloco            = %.6f  [s]\n', dt_bloco);
% fprintf('  dt_ik               = %.6f  [s]\n', dt_ik);
% 
% % -----------------------------
% % E) Controle PD — base (Euler)
% % -----------------------------
% fprintf('\n[E] Controle PD — base (modo não operacional: Euler)\n');
% fprintf('  ControlePD_tau_Euler   = %.6f\n', ControlePD_tau_Euler);
% fprintf('  PD_tau_Euler           = [%.6f; %.6f; %.6f]\n', PD_tau_Euler(1), PD_tau_Euler(2), PD_tau_Euler(3));
% fprintf('  Euler_Kprop            = %.6f\n', Euler_Kprop);
% fprintf('  Euler_Kder             = %.6f\n', Euler_Kder);
% fprintf('  Euler_PD_Kprop         = [%.6f; %.6f; %.6f]\n', Euler_PD_Kprop(1), Euler_PD_Kprop(2), Euler_PD_Kprop(3));
% fprintf('  Euler_PD_Kder          = [%.6f; %.6f; %.6f]\n', Euler_PD_Kder(1), Euler_PD_Kder(2), Euler_PD_Kder(3));
% 
% % -----------------------------
% % F) Controle PD — base (Lagrange)
% % -----------------------------
% fprintf('\n[F] Controle PD — base (modo operacional: Lagrange)\n');
% fprintf('  ControlePD_tau_Lagrange= %.6f\n', ControlePD_tau_Lagrange);
% fprintf('  PD_tau_Lagrange        = [%.6f; %.6f; %.6f]\n', PD_tau_Lagrange(1), PD_tau_Lagrange(2), PD_tau_Lagrange(3));
% fprintf('  Lagrange_Kprop         = %.6f\n', Lagrange_Kprop);
% fprintf('  Lagrange_Kder          = %.6f\n', Lagrange_Kder);
% fprintf('  Lagrange_PD_Kprop      = [%.6f; %.6f; %.6f]\n', Lagrange_PD_Kprop(1), Lagrange_PD_Kprop(2), Lagrange_PD_Kprop(3));
% fprintf('  Lagrange_PD_Kder       = [%.6f; %.6f; %.6f]\n', Lagrange_PD_Kder(1), Lagrange_PD_Kder(2), Lagrange_PD_Kder(3));
% 
% % -----------------------------
% % G) Controle PD — manipulador (3R)
% % -----------------------------
% fprintf('\n[G] Controle PD — manipulador (3R)\n');
% fprintf('  MR_Kprop               = %.6f\n', MR_Kprop);
% fprintf('  MR_Kder                = %.6f\n', MR_Kder);
% fprintf('  MR_sat                 = %.6f\n', MR_sat);
% fprintf('  MR_tau                 = %.6f\n', MR_tau);
% fprintf('  KFF                    = [%.6f; %.6f; %.6f]\n', KFF(1), KFF(2), KFF(3));
% fprintf('  Kp_tarefa              = %.6f\n', Kp_tarefa);
% fprintf('  Kp_angulo              = %.6f\n', Kp_angulo);
% fprintf('  modo_op_radial         = %.6f\n', modo_op_radial);
% fprintf('  modo_op_tempo          = %.6f  [s]\n', modo_op_tempo);
% fprintf('  Kp_pos                 = %.6f\n', Kp_pos);
% fprintf('  Kp_ori                 = %.6f\n', Kp_ori);
% fprintf('  limite_veloc_junta     = %.6f  [rad/s]\n', limite_veloc_junta);
% fprintf('  lambda                 = %.6f\n', lambda);
% fprintf('  ControlePD_MR_Kprop    = [%.6f; %.6f; %.6f]\n', ControlePD_MR_Kprop(1), ControlePD_MR_Kprop(2), ControlePD_MR_Kprop(3));
% fprintf('  ControlePD_MR_Kder     = [%.6f; %.6f; %.6f]\n', ControlePD_MR_Kder(1), ControlePD_MR_Kder(2), ControlePD_MR_Kder(3));
% fprintf('  ControlePD_MR_tau_max  = [%.6f; %.6f; %.6f]\n', ControlePD_MR_tau_max(1), ControlePD_MR_tau_max(2), ControlePD_MR_tau_max(3));
% 
% % -----------------------------
% % H) Controle por twist (quando habilitado)
% % -----------------------------
% fprintf('\n[H] Controle por twist (quando habilitado)\n');
% fprintf('  twist_k_ang            = %.6f\n', twist_k_ang);
% fprintf('  twist_k_lat            = %.6f\n', twist_k_lat);
% fprintf('  twist_k_ax             = %.6f\n', twist_k_ax);
% fprintf('  twist_eps_lat          = %.6f\n', twist_eps_lat);
% fprintf('  twist_eps_graus        = %.6f  [graus]\n', twist_eps_graus);
% fprintf('  twist_sat_veloc_ref    = %.6f\n', twist_sat_veloc_ref);
% 
% % -----------------------------
% % I) Condições iniciais
% % -----------------------------
% fprintf('\n[I] Condições iniciais\n');
% fprintf('  base_pos_inicial           = [%.6f; %.6f; %.6f]\n', base_pos_inicial(1), base_pos_inicial(2), base_pos_inicial(3));
% fprintf('  base_veloc_inicial         = [%.6f; %.6f; %.6f]\n', base_veloc_inicial(1), base_veloc_inicial(2), base_veloc_inicial(3));
% fprintf('  base_quaternion_inicial    = [%.6f; %.6f; %.6f; %.6f]\n', ...
%     base_quaternion_inicial(1), base_quaternion_inicial(2), base_quaternion_inicial(3), base_quaternion_inicial(4));
% fprintf('  base_veloc_angular_inicial = [%.6f; %.6f; %.6f]\n', ...
%     base_veloc_angular_inicial(1), base_veloc_angular_inicial(2), base_veloc_angular_inicial(3));
% 
% fprintf('  juntas_pos_inicial         = [%.6f; %.6f; %.6f]\n', juntas_pos_inicial(1), juntas_pos_inicial(2), juntas_pos_inicial(3));
% fprintf('  juntas_veloc_inicial       = [%.6f; %.6f; %.6f]\n', juntas_veloc_inicial(1), juntas_veloc_inicial(2), juntas_veloc_inicial(3));
% fprintf('  juntas_pos_desejada        = [%.6f; %.6f; %.6f]\n', juntas_pos_desejada(1), juntas_pos_desejada(2), juntas_pos_desejada(3));
% fprintf('  juntas_veloc_desejada      = [%.6f; %.6f; %.6f]\n', ...
%     juntas_veloc_desejada(1), juntas_veloc_desejada(2), juntas_veloc_desejada(3));
% fprintf('  juntas_limite_veloc        = [%.6f; %.6f; %.6f]\n', juntas_limite_veloc(1), juntas_limite_veloc(2), juntas_limite_veloc(3));
% 
% fprintf('\n[J] Exportação para workspace\n');
% fprintf('  Variáveis exportadas: x0, tspan, tempo_final, delta_R0, delta_V0, omega_orb, massas_elos, estados, params_flat\n');
% 
% fprintf('\n=====================================================================\n');
% fprintf('FIM DO RELATÓRIO\n');
% fprintf('=====================================================================\n\n');
