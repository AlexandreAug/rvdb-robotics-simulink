%% Main.m

%% ===== Caminhos e parâmetros de simulação
% addpath("Funcoes");
addpath(pwd);

% Parâmetros
Parametros_perseguidor;
ParametrosEHCW;

%% ===== Transferência orbital
TH = TransferenciaOrbital();
[r_t0, v_t0, r_c0, v_c0, r_tF, v_tF, r_cF, v_cF] = deal( ...
    TH.eci_target_inicial_raio,  TH.eci_target_inicial_veloc, ...
    TH.eci_chaser_inicial_raio,  TH.eci_chaser_inicial_veloc, ...
    TH.eci_target_final_raio,    TH.eci_target_final_veloc, ...
    TH.eci_chaser_final_raio,    TH.eci_chaser_final_veloc);

assignin('base','TH',TH);

%% ===== Eq. de Hill/CW
[A_xz, B_xz, C_xz, D_xz, A_y, B_y, C_y, D_y, w] = ...
    Fun_HillCW(TH.raio_target, Constantes.mu_Terra, massa_escalar_total_chaser);

objetivo_Hill_radial = Constantes.delta_radial_Hill;
objetivo_Hill_alongtrack = Constantes.delta_alongtrack_Hill;
objetivo_Hill_crosstrack = Constantes.delta_crosstrack_Hill;

assignin('base','A_xz',A_xz);
assignin('base','B_xz',B_xz);
assignin('base','C_xz',C_xz);
assignin('base','D_xz',D_xz);
assignin('base','A_y', A_y);
assignin('base','B_y', B_y);
assignin('base','C_y', C_y);
assignin('base','D_y', D_y);
assignin('base','w',   w);
assignin('base','objetivo_Hill_radial',objetivo_Hill_radial);
assignin('base','objetivo_Hill_alongtrack',objetivo_Hill_alongtrack);
assignin('base','objetivo_Hill_crosstrack',objetivo_Hill_crosstrack);

% Condições iniciais em LVLH
condicoes_iniciais_radialAlongtrack = [TH.radial_drift; TH.alongtrack_drift; TH.vradial_drift; TH.valongtrack_drift];
condicoes_iniciais_crosstrack       = [TH.crosstrack_drift; TH.vcrosstrack_drift];

assignin('base','X0_xz',condicoes_iniciais_radialAlongtrack);
assignin('base','X0_y', condicoes_iniciais_crosstrack);

%% ===== GNC / Atitude

% Definição da fase atual do GNC
fase_GNC = 'HILL';

assignin('base','fase_GNC',fase_GNC); 
assignin('base','attd_target_quaternion',attd_target_quaternion);
assignin('base','attd_inicial_chaser_quaternion',attd_inicial_chaser_quaternion);
assignin('base','attd_chaser_desejada_quaternion',attd_chaser_desejada_quaternion);

assignin('base','attd_velocAngular_inicial_chaser',attd_velocAngular_inicial_chaser);
assignin('base','attd_velocAngular_desejada_chaser',attd_velocAngular_desejada_chaser);

assignin('base','attd_MI_chaser_home',attd_MI_chaser_home);

assignin('base','attd_Kprop',attd_Kprop);
assignin('base','attd_Kder',attd_Kder);
assignin('base','attd_tau_max',attd_tau_max);

