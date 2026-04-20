function TransfOrbital = Graficos_1_Transferencia_Orbital()
% --------------------------------------------------------------------------
% Graficos_1_Transferencia_Orbital.m
% --------------------------------------------------------------------------
    clc; 
    % ======= Constantes e condições iniciais (SI) ==========================
    mu         = Constantes.mu_Terra;              % [m^3/s^2]
    raio_Terra = Constantes.raio_Terra;            % [m]

    % Longitudes inerciais iniciais (deg -> rad)
    longitude_chaser_inicial = deg2rad(Constantes.longitude_inercial_chaser_inicial);  % [rad]
    longitude_chaser_inicial_perturbado = deg2rad(Constantes.longitude_inercial_chaser_inicial_perturbado);  % [rad]

    longitude_target_inicial = deg2rad(Constantes.longitude_inercial_target_inicial);  % [rad]

    % Raios orbitais (circulares, coplanares)
    raio_chaser_inicial = raio_Terra + Constantes.altitude_chaser_inicial;   % [m

    raio_chaser_inicial_perturbado = raio_Terra + Constantes.altitude_chaser_inicial + ...
                                     Constantes.ECI_erro_injecao_chaser_radial;   % [m]

    raio_target         = raio_Terra + Constantes.altitude_target_inicial;   % [m]
    
    % ======= Estados ECI iniciais (via OEs) ================================
    e = 0; inc = 0; RAAN = 0; argp = 0;
    nu_t0 = longitude_target_inicial;   % anomalia verdadeira = longitude (coplanar circular)
    nu_c0 = longitude_chaser_inicial;
    
    a_t0 = raio_target;
    a_c0 = raio_chaser_inicial;
    
    [eci_target_inicial_raio,  eci_target_inicial_veloc]  = oe2eci(mu, a_t0, e, inc, RAAN, argp, nu_t0);
    [eci_chaser_inicial_raio,  eci_chaser_inicial_veloc]  = oe2eci(mu, a_c0, e, inc, RAAN, argp, nu_c0);

    % Offsets desejados após TH, medidos no LVLH do alvo
    offset_radial     = Constantes.delta_radial_TH;     % [m] (z LVLH: -radial, para dentro da Terra)
    offset_alongtrack = Constantes.delta_alongtrack_TH; % [m] (x LVLH: +na direção da velocidade do alvo)
    offset_crosstrack = Constantes.delta_crosstrack_TH; % [m] (y LVLH: -normal ao plano)

    % Raio final desejado do chaser após TH (circularização na queima 2)
    raio_chaser_final = raio_target + offset_radial;    % [m]

    % Check: transferência deve ser de inferior -> superior
    if raio_chaser_final <= raio_chaser_inicial
        error('Esta função assume transferência de órbita inferior -> superior. r_final(=rt+dx) deve ser > r_chaser0.');
    end

    % ======= Cinemática =============================================
    n_chaser_inicial = sqrt(mu / raio_chaser_inicial^3); % [rad/s]
    n_target         = sqrt(mu / raio_target^3);         % [rad/s]

    % Órbita de transferência (Elíptica)
    orbita_transf = 0.5 * (raio_chaser_inicial + raio_chaser_final); % [m]
    tempo_transf  = pi * sqrt(orbita_transf^3 / mu);                 % [s]

    % Δvs da Hohmann
    veloc_circ_chaser_inicial = sqrt(mu / raio_chaser_inicial); % [m/s]
    veloc_circ_chaser_final   = sqrt(mu / raio_chaser_final);   % [m/s]

    veloc_perigeu_transf = sqrt(2*mu*raio_chaser_final/...
        (raio_chaser_inicial*(raio_chaser_inicial + raio_chaser_final))); % [m/s]
    veloc_apogeu_transf  = sqrt(2*mu*raio_chaser_inicial/...
        (raio_chaser_final*(raio_chaser_inicial + raio_chaser_final)));   % [m/s]

    deltav1      = veloc_perigeu_transf - veloc_circ_chaser_inicial; % [m/s]
    deltav2      = veloc_circ_chaser_final - veloc_apogeu_transf;    % [m/s]
    deltav_total = abs(deltav1) + abs(deltav2);                     % [m/s]

    % ======= Fase desejada ao final da TH (instante da Δv2) ================
    phi_desejado = - offset_alongtrack / raio_target; % [rad]

    % Fase inicial (0..2π)
    phi_inicial = wrapTo2Pi(longitude_target_inicial - longitude_chaser_inicial); % [rad]

    % Evolução da fase
    denom = (n_target - n_chaser_inicial);  % [rad/s]
    if abs(denom) < 1e-12
        error('n_t e n_c0 quase iguais – condição de fase mal condicionada.');
    end

    tempo_espera_req = (phi_desejado - phi_inicial - (n_target*tempo_transf - pi)) / denom; % [s]
    % Normaliza para o 1º valor não-negativo (somando múltiplos do período sinódico)
    T_syn = 2*pi / abs(denom);     % [s] período sinódico
    tempo_espera = tempo_espera_req;
    if tempo_espera < 0
        k = ceil(-tempo_espera / T_syn);
        tempo_espera = tempo_espera + k*T_syn; % [s]
    end

    % ======= Histereses para não ocorrer manobra imediata
    % Opção A: histerese temporal mínima
    tempo_espera_min = 5; % [s]
    % Opção B: Tolerância de fase
    fase_tol_deg = 1;
    fase_tol_rad = deg2rad(fase_tol_deg);

    % Se começasse AGORA (tempo_espera = 0), qual φ ao final da TH?
    longitude_target_se_queima2_agora = wrapTo2Pi(longitude_target_inicial + n_target*tempo_transf);
    longitude_chaser_se_queima1_agora = wrapTo2Pi(longitude_chaser_inicial + pi); % varre π na TH
    phi_se_agora = wrapToPi(longitude_target_se_queima2_agora - longitude_chaser_se_queima1_agora);

    % Aplicando as regras (sem somar 2x o T_syn)
    pulou = false;
    if tempo_espera < tempo_espera_min
        tempo_espera = tempo_espera + T_syn;
        pulou = true;
    end
    if ~pulou && abs(wrapToPi(phi_se_agora - phi_desejado)) <= fase_tol_rad
        tempo_espera = tempo_espera + T_syn;
        pulou = true;
    end

    % ====== Taxas angulares para PROPAGAÇÃO com perturbação ================
    try eps_target = Constantes.perturbacao_target; catch, eps_target = 0; end
    try eps_chaser = Constantes.ECI_perturbacao_chaser_y; catch, eps_chaser = 0; end

    n_target_propagado = n_target * (1 + eps_target);
    n_chaser_propagado = n_chaser_inicial * (1 + eps_chaser);

    % ======= Longitudes em Δv1 e Δv2 com perturbação =======================
    longitude_chaser_burn1 = wrapTo2Pi(longitude_chaser_inicial + n_chaser_propagado*tempo_espera);
    longitude_target_burn1 = wrapTo2Pi(longitude_target_inicial + n_target_propagado*tempo_espera);

    tempo_burn2            = tempo_espera + tempo_transf;
    longitude_target_burn2 = wrapTo2Pi(longitude_target_burn1 + n_target_propagado*tempo_transf);
    longitude_chaser_burn2 = wrapTo2Pi(longitude_chaser_burn1 + pi);  % chaser varre π na TH

    % Fase alcançada (diagnóstico): φ2 = (θ_t - θ_c) em b2
    phi_final = wrapToPi(longitude_target_burn2 - longitude_chaser_burn2);

    % ======= Estados ECI iniciais ==========================================
    eci_target_inicial_raio  = raio_target * [cos(longitude_target_inicial); sin(longitude_target_inicial); 0];
    eci_target_inicial_veloc = sqrt(mu / raio_target) * [-sin(longitude_target_inicial); cos(longitude_target_inicial); 0];

    eci_chaser_inicial_raio  = raio_chaser_inicial * [cos(longitude_chaser_inicial); sin(longitude_chaser_inicial); 0];
    eci_chaser_inicial_veloc = sqrt(mu / raio_chaser_inicial) * [-sin(longitude_chaser_inicial); cos(longitude_chaser_inicial); 0];

    % ======= Estados ECI finais (no instante da queima 2) ===================
    % eci_target_final_raio  = raio_target * [cos(longitude_target_burn2);  ...
    %                                         sin(longitude_target_burn2); 0];
    % eci_target_final_veloc = sqrt(mu / raio_target) * [ -sin(longitude_target_burn2); ...
    %                                                      cos(longitude_target_burn2); 0];
    % 
    % % Chaser (após circularização na órbita final desejada)
    % eci_chaser_final_raio  = raio_chaser_final * [cos(longitude_chaser_burn2); ...
    %                                               sin(longitude_chaser_burn2); 0];
    % eci_chaser_final_veloc = sqrt(mu / raio_chaser_final) * [ -sin(longitude_chaser_burn2); ...
                                                               % cos(longitude_chaser_burn2); 0];

    % ======= Estados ECI finais (no instante da Δv2) via OEs ===============
    nu_t2 = longitude_target_burn2;
    nu_c2 = longitude_chaser_burn2;
    
    a_t2 = raio_target;        % alvo continua circular em r_target
    a_c2 = raio_chaser_final;  % chaser circularizou em r_final
    
    [eci_target_final_raio,  eci_target_final_veloc]  = oe2eci(mu, a_t2, e, inc, RAAN, argp, nu_t2);
    [eci_chaser_final_raio,  eci_chaser_final_veloc]  = oe2eci(mu, a_c2, e, inc, RAAN, argp, nu_c2);

    % ===== Drift pós-TH e amostragem para iniciar Hill/CW =====
    dt_drift = Constantes.tempo_drift;  % [s] x minutos de drift
    t_drift  = tempo_burn2 + dt_drift;  % instante de amostragem
    
    % Taxas finais (circular) para o pós-TH
    n_target_drift       = sqrt(mu / raio_target^3);
    n_chaser_drift       = sqrt(mu / raio_chaser_final^3);
    
    % Aplicando as perturbações
    n_target_final_propagado  = n_target_drift  * (1 + eps_target);
    n_chaser_final_propagado  = n_chaser_drift  * (1 + eps_chaser);
    
    % Longitudes no instante de amostragem t_drift
    theta_t_drift = wrapTo2Pi(longitude_target_burn2 + n_target_final_propagado * dt_drift);
    theta_c_drift = wrapTo2Pi(longitude_chaser_burn2 + n_chaser_final_propagado * dt_drift);
    
    % ECI em t_drift
    % eci_target_drift_r = raio_target * [cos(theta_t_drift);  sin(theta_t_drift);  0];
    % eci_chaser_drift_r = raio_chaser_final * [cos(theta_c_drift); sin(theta_c_drift); 0];
    % 
    % eci_target_drift_v = sqrt(mu/raio_target)       * [-sin(theta_t_drift);  cos(theta_t_drift);  0];
    % eci_chaser_drift_v = sqrt(mu/raio_chaser_final) * [-sin(theta_c_drift);  cos(theta_c_drift);  0];

    % ECI em t_drift via COEs
    nu_t_drift = theta_t_drift;   % já calculado no seu bloco de drift
    nu_c_drift = theta_c_drift;
    
    a_t_drift = raio_target;        % circular
    a_c_drift = raio_chaser_final;  % circular
    
    [eci_target_drift_r, eci_target_drift_v] = oe2eci(mu, a_t_drift, e, inc, RAAN, argp, nu_t_drift);
    [eci_chaser_drift_r, eci_chaser_drift_v] = oe2eci(mu, a_c_drift, e, inc, RAAN, argp, nu_c_drift);
    oe_target_drift = rv2oe(mu, eci_target_drift_r, eci_target_drift_v);
    oe_chaser_drift = rv2oe(mu, eci_chaser_drift_r, eci_chaser_drift_v);

    % ======= Conversão ECI -> LVLH do alvo no instante da Δv2 ==============
    r_t = eci_target_final_raio;    % [m]
    v_t = eci_target_final_veloc;   % [m/s]
    r_c = eci_chaser_final_raio;    % [m]
    v_c = eci_chaser_final_veloc;   % [m/s]

    r_t_s = eci_target_drift_r;    v_t_s = eci_target_drift_v;
    r_c_s = eci_chaser_drift_r;    v_c_s = eci_chaser_drift_v;

    % Base LVLH (dv2): x=r̂, y=t̂, z=n̂
    Rhat = r_t / norm(r_t);
    hvec = cross(r_t, v_t);
    Nhat = hvec / norm(hvec);
    That = cross(Nhat, Rhat);
    C_eci2lvlh = [Rhat.'; That.'; Nhat.'];     % 3x3
    % Base LVLH (após drift): x=r̂, y=t̂, z=n̂
    Rhat = r_t_s / norm(r_t_s);
    hvec = cross(r_t_s, v_t_s);  Nhat = hvec / norm(hvec);
    That = cross(Nhat, Rhat);
    C_eci2lvlh_s = [Rhat.'; That.'; Nhat.'];

    % Relativo em ECI (após dv2)
    r_rel_eci = r_c - r_t;
    v_rel_eci = v_c - v_t;
    % Relativo em ECI (após drift)
    r_rel_eci_s = r_c_s - r_t_s;
    v_rel_eci_s = v_c_s - v_t_s;

    % Posição e velocidade relativa em LVLH (após dv2)
    r_rel_lvlh = C_eci2lvlh * r_rel_eci;       % [x; y; z]
    w  = sqrt(mu / norm(r_t)^3);               % [rad/s]
    Om = [0;0;w];
    v_rel_lvlh = C_eci2lvlh * v_rel_eci - cross(Om, r_rel_lvlh);  % [vx; vy; vz]
    % Posição e velocidade relativa em LVLH (após drift)
    r_rel_lvlh_s = C_eci2lvlh_s * r_rel_eci_s;
    w_s          = sqrt(mu / norm(r_t_s)^3);
    Om_s         = [0;0;w_s];
    v_rel_lvlh_s = C_eci2lvlh_s * v_rel_eci_s - cross(Om_s, r_rel_lvlh_s);

    % componentes LVLH (após dv2)
    radial_b2     = r_rel_lvlh(1); % radial
    alongtrack_b2 = r_rel_lvlh(2); % along
    crosstrack_b2 = r_rel_lvlh(3); % cross
    
    vradial_b2 = v_rel_lvlh(1);
    valongtrack_b2 = v_rel_lvlh(2);
    vcrosstrack_b2 = v_rel_lvlh(3);

    % componentes LVLH (após drift)
    radial_drift  = r_rel_lvlh_s(1)     * (1 + Constantes.relativo_perturbacao_chaser_radial);
    alongtrack_drift  = r_rel_lvlh_s(2) * (1 + Constantes.relativo_perturbacao_chaser_alongtrack);
    crosstrack_drift  = r_rel_lvlh_s(3) * (1 + Constantes.relativo_perturbacao_chaser_crosstrack);

    vradial_drift = v_rel_lvlh_s(1)     * (1 + Constantes.relativo_perturbacao_chaser_vradial);
    valongtrack_drift = v_rel_lvlh_s(2) * (1 + Constantes.relativo_perturbacao_chaser_valongtrack);
    vcrosstrack_drift = v_rel_lvlh_s(3) * (1 + Constantes.relativo_perturbacao_chaser_vcrosstrack);
    
    % ===== Avaliações de erro e tolerância (usando LVLH direto) =====
    % Fase (rad)
    tol_fase  = 0.01 * abs(phi_desejado);
    erro_fase = abs(phi_final - phi_desejado);
    if phi_desejado == 0
        dentro_fase = (phi_final == 0); folga_fase = NaN;
    else
        dentro_fase = (erro_fase <= tol_fase);
        folga_fase  = max(0, (tol_fase - erro_fase)/tol_fase);
    end
    status_fase = 'OK'; if ~dentro_fase, status_fase = 'FORA'; end
    % Radial (z)
    radial_des_m = offset_radial;
    radial_fin_m = radial_b2;
    tol_radial   = 0.01 * abs(radial_des_m);
    err_radial   = abs(radial_fin_m - radial_des_m);
    if radial_des_m == 0
        dentro_radial = (abs(radial_fin_m) == 0); folga_radial = NaN;
    else
        dentro_radial = (err_radial <= tol_radial);
        folga_radial  = max(0, (tol_radial - err_radial)/tol_radial);
    end
    status_radial = 'OK'; if ~dentro_radial, status_radial = 'FORA'; end
    
    % Along-track (x)
    along_des_m = offset_alongtrack;   % < 0 para "atrás"
    along_fin_m = alongtrack_b2;                  % usa LVLH direto
    tol_along   = 0.01 * abs(along_des_m);
    err_along   = abs(along_fin_m - along_des_m);
    if along_des_m == 0
        dentro_along = (abs(along_fin_m) == 0); folga_along = NaN;
    else
        dentro_along = (err_along <= tol_along);
        folga_along  = max(0, (tol_along - err_along)/tol_along);
    end
    status_along = 'OK'; if ~dentro_along, status_along = 'FORA'; end
    
    % Cross-track (y)
    crosstrack_des_m = offset_crosstrack;
    crosstrack_fin_m = crosstrack_b2;
    if crosstrack_des_m == 0
        tol_cross = 0;
        err_cross = abs(crosstrack_fin_m); 
        folga_cross_str = '0';
    else
        tol_cross = 0.01 * abs(crosstrack_des_m);
        err_cross = abs(crosstrack_fin_m - crosstrack_des_m);
        folga_cross = max(0, (tol_cross - err_cross)/tol_cross);
        folga_cross_str = sprintf('%.2f%%', 100*folga_cross);
    end
    if err_cross <= tol_cross
        status_cross = 'OK'; 
    else
        status_cross = 'FORA';
    end

    % ======= Relatório =======
    se   = @(x) sprintf('%.0f', x);
    sh   = @(x) sprintf('%.2f', x/3600);
    shms = @(x) fmtHMS(x);
    pct  = @(x) sprintf('%.2f%%', 100*x);

    % ======= Estados ECI no instante da Δv1 (fim da espera) ===================
nu_t1 = longitude_target_burn1;
nu_c1 = longitude_chaser_burn1;

a_t1  = raio_target;         % alvo continua circular
a_c1  = raio_chaser_inicial; % chaser ainda está na órbita inicial (antes da queima)

[eci_target_burn1_raio,  eci_target_burn1_veloc] = oe2eci(mu, a_t1, e, inc, RAAN, argp, nu_t1);
[eci_chaser_burn1_raio,  eci_chaser_burn1_veloc] = oe2eci(mu, a_c1, e, inc, RAAN, argp, nu_c1);


    %% ======= Pacote de saída ===========================
    TransfOrbital = struct();
    
    % Geral
    TransfOrbital.tempo_espera             = tempo_espera;

    TransfOrbital.tempo_transf             = tempo_transf;
    TransfOrbital.tempo_burn2              = tempo_burn2;

    % DeltaV
    TransfOrbital.deltav1                  = deltav1;
    TransfOrbital.deltav2                  = deltav2;
    TransfOrbital.deltav_total             = deltav_total;

    % Raios orbitais
    TransfOrbital.raio_chaser_inicial      = raio_chaser_inicial;
    TransfOrbital.raio_target              = raio_target;
    TransfOrbital.raio_chaser_final        = raio_chaser_final;
    TransfOrbital.phi_inicial              = phi_inicial;
    TransfOrbital.fase_desejada_phi        = phi_desejado;
    TransfOrbital.fase_alcancada_phi2      = phi_final;

    % Longitude
    TransfOrbital.longitude_chaser_burn1   = longitude_chaser_burn1;
    TransfOrbital.longitude_target_burn1   = longitude_target_burn1;

    TransfOrbital.longitude_target_burn2   = longitude_target_burn2;
    TransfOrbital.longitude_chaser_burn2   = longitude_chaser_burn2;
% fprintf("Longitude perseguidor queima 1: %.8f \n", longitude_chaser_burn1);
% fprintf("Longitude alvo queima 1: %.8f \n", longitude_target_burn1);
    % fprintf("Longitude perseguidor queima 2: %.12f \n", longitude_chaser_burn2);
    % fprintf("Longitude alvo queima 2: %.12f \n", longitude_target_burn2);


    %% ECI (início da simulação)
    TransfOrbital.eci_target_inicial_raio  = eci_target_inicial_raio;
    TransfOrbital.eci_target_inicial_veloc = eci_target_inicial_veloc;
        TransfOrbital.eci_chaser_inicial_raio  = eci_chaser_inicial_raio;
        TransfOrbital.eci_chaser_inicial_veloc = eci_chaser_inicial_veloc;

% fprintf('eci_chaser_inicial_raio = [%g; %g; %g]\n', eci_chaser_inicial_raio(1)/1e3, eci_chaser_inicial_raio(2)/1e3, eci_chaser_inicial_raio(3)/1e3);
% fprintf('eci_chaser_inicial_veloc = [%g; %g; %g]\n', eci_chaser_inicial_veloc(1)/1e3, eci_chaser_inicial_veloc(2)/1e3, eci_chaser_inicial_veloc(3)/1e3);
%     fprintf('eci_target_inicial_raio = [%g; %g; %g]\n', eci_target_inicial_raio(1)/1e3, eci_target_inicial_raio(2)/1e3, eci_target_inicial_raio(3)/1e3);
%     fprintf('eci_target_inicial_veloc = [%g; %g; %g]\n', eci_target_inicial_veloc(1)/1e3, eci_target_inicial_veloc(2)/1e3, eci_target_inicial_veloc(3)/1e3);


    %% ECI (instante de Δv1)
    TransfOrbital.eci_target_burn1_raio  = eci_target_burn1_raio;
    TransfOrbital.eci_target_burn1_veloc = eci_target_burn1_veloc;
        TransfOrbital.eci_chaser_burn1_raio  = eci_chaser_burn1_raio;
        TransfOrbital.eci_chaser_burn1_veloc = eci_chaser_burn1_veloc;

% fprintf('eci_target_burn1_raio = [%g; %g; %g]\n',  eci_target_burn1_raio(1)/1e3,  eci_target_burn1_raio(2)/1e3,  eci_target_burn1_raio(3)/1e3);
% fprintf('eci_target_burn1_veloc = [%g; %g; %g]\n', eci_target_burn1_veloc(1)/1e3, eci_target_burn1_veloc(2)/1e3, eci_target_burn1_veloc(3)/1e3);
%     fprintf('eci_chaser_burn1_raio = [%g; %g; %g]\n',  eci_chaser_burn1_raio(1)/1e3,  eci_chaser_burn1_raio(2)/1e3,  eci_chaser_burn1_raio(3)/1e3);
%     fprintf('eci_chaser_burn1_veloc = [%g; %g; %g]\n', eci_chaser_burn1_veloc(1)/1e3, eci_chaser_burn1_veloc(2)/1e3, eci_chaser_burn1_veloc(3)/1e3);

    %% ECI (instante de Δv2)
    TransfOrbital.eci_chaser_final_raio  = eci_chaser_final_raio;
    TransfOrbital.eci_chaser_final_veloc = eci_chaser_final_veloc;
        TransfOrbital.eci_target_final_raio  = eci_target_final_raio;
        TransfOrbital.eci_target_final_veloc = eci_target_final_veloc;

% fprintf('eci_chaser_final_raio = [%g; %g; %g]\n', eci_chaser_final_raio(1)/1e3, eci_chaser_final_raio(2)/1e3, eci_chaser_final_raio(3)/1e3);
% fprintf('eci_chaser_final_veloc = [%g; %g; %g]\n', eci_chaser_final_veloc(1)/1e3, eci_chaser_final_veloc(2)/1e3, eci_chaser_final_veloc(3)/1e3);
    % fprintf('eci_target_final_raio = [%g; %g; %g]\n', eci_target_final_raio(1)/1e3, eci_target_final_raio(2)/1e3, eci_target_final_raio(3)/1e3);
    % fprintf('eci_target_final_veloc = [%g; %g; %g]\n', eci_target_final_veloc(1)/1e3, eci_target_final_veloc(2)/1e3, eci_target_final_veloc(3)/1e3);
    

    %% ECI (depois do drift)
    TransfOrbital.eci_chaser_final_drift_raio  = eci_chaser_drift_r;
    TransfOrbital.eci_chaser_final_drift_veloc = eci_chaser_drift_v;
    TransfOrbital.eci_target_final_drift_raio  = eci_target_drift_r;
    TransfOrbital.eci_target_final_drift_veloc = eci_target_drift_v;

% fprintf('eci_chaser_final_drift_raio = [%g; %g; %g]\n', eci_chaser_drift_r(1)/1e3, eci_chaser_drift_r(2)/1e3, eci_chaser_drift_r(3)/1e3);
% fprintf('eci_chaser_final_drift_veloc = [%g; %g; %g]\n', eci_chaser_drift_v(1)/1e3, eci_chaser_drift_v(2)/1e3, eci_chaser_drift_v(3)/1e3);
    % fprintf('eci_target_final_drift_raio = [%g; %g; %g]\n', eci_target_drift_r(1)/1e3, eci_target_drift_r(2)/1e3, eci_target_drift_r(3)/1e3);
    % fprintf('eci_target_final_drift_veloc = [%g; %g; %g]\n', eci_target_drift_v(1)/1e3, eci_target_drift_v(2)/1e3, eci_target_drift_v(3)/1e3);

    %% OEs (depois do drift)
    TransfOrbital.oe_target_drift = oe_target_drift;
    TransfOrbital.oe_chaser_drift = oe_chaser_drift;

fprintf('oe_target_drift: a=%g km, e=%g, inc=%g deg, RAAN=%g deg, argp=%g deg, nu=%g deg\n', ...
    oe_target_drift.a/1e3, oe_target_drift.e, ...
    rad2deg(oe_target_drift.inc), rad2deg(oe_target_drift.RAAN), ...
    rad2deg(oe_target_drift.argp), rad2deg(oe_target_drift.nu));

fprintf('oe_chaser_drift: a=%g km, e=%g, inc=%g deg, RAAN=%g deg, argp=%g deg, nu=%g deg\n', ...
    oe_chaser_drift.a/1e3, oe_chaser_drift.e, ...
    rad2deg(oe_chaser_drift.inc), rad2deg(oe_chaser_drift.RAAN), ...
    rad2deg(oe_chaser_drift.argp), rad2deg(oe_chaser_drift.nu));
        
    %% Período orbital
    TransfOrbital.T_syn                    = T_syn;
    TransfOrbital.t_espera_min             = tempo_espera_min;
    TransfOrbital.fase_tol_deg             = fase_tol_deg;

    % LVLH e métricas
    TransfOrbital.alongtrack_desejado_m   = along_des_m;
    TransfOrbital.alongtrack_alcancado_m  = along_fin_m;
    TransfOrbital.radial_desejado_m       = radial_des_m;
    TransfOrbital.radial_alcancado_m      = radial_fin_m;
    TransfOrbital.crosstrack_desejado_m   = crosstrack_des_m;
    TransfOrbital.crosstrack_alcancado_m  = crosstrack_fin_m;
    
    % Erros e tolerâncias
    TransfOrbital.err_fase_rad            = erro_fase;
    TransfOrbital.tol_fase_rad            = tol_fase;
    TransfOrbital.err_along_m             = err_along;
    TransfOrbital.tol_along_m             = tol_along;
    TransfOrbital.err_radial_m            = err_radial;
    TransfOrbital.tol_radial_m            = tol_radial;
    TransfOrbital.err_crosstrack_m        = err_cross;
    TransfOrbital.tol_crosstrack_m        = tol_cross;

    %% LVLH (no instante Δv2)
    TransfOrbital.lvlh_rel_pos_b2 = r_rel_lvlh;   % [x;y;z] (m)
    % fprintf("LVLH perseguidor na queima 2: %.2f, %.2f, %.2f \n", r_rel_lvlh);
    TransfOrbital.lvlh_rel_vel_b2 = v_rel_lvlh;   % [vx;vy;vz] (m/s)

    TransfOrbital.radial_b2     = radial_b2;
    TransfOrbital.alongtrack_b2 = alongtrack_b2;
    TransfOrbital.crosstrack_b2 = crosstrack_b2;

    TransfOrbital.vradial_b2     = vradial_b2;
    TransfOrbital.valongtrack_b2 = valongtrack_b2;
    TransfOrbital.vcrosstrack_b2 = vcrosstrack_b2;

    %% LVLH (após drift)
    TransfOrbital.t_drift  = t_drift;
    TransfOrbital.dt_drift = dt_drift;
    TransfOrbital.lvlh_rel_pos_drift = r_rel_lvlh_s;
    % fprintf("LVLH perseguidor após drift: %.2f, %.2f, %.2f \n", r_rel_lvlh_s);
    TransfOrbital.lvlh_rel_vel_drift = v_rel_lvlh_s;
    % fprintf("LVLH velocidade perseguidor após drift: %.2f, %.2f, %.2f \n", v_rel_lvlh_s);


    TransfOrbital.radial_drift     = radial_drift;
    TransfOrbital.alongtrack_drift = alongtrack_drift;
    TransfOrbital.crosstrack_drift = crosstrack_drift;

    TransfOrbital.vradial_drift     = vradial_drift;
    TransfOrbital.valongtrack_drift = valongtrack_drift;
    TransfOrbital.vcrosstrack_drift = vcrosstrack_drift;

    %% GRAFICOS
    % ======================================================================

fs = 16;  % tamanho da fonte para eixos, títulos e legendas

set(groot,'defaultAxesFontSize',   fs);   % eixos (ticks, x/y)
set(groot,'defaultTextFontSize',   fs);   % textos em geral
set(groot,'defaultLegendFontSize', fs);   % legendas

    % --------- amostragem temporal (espera + transferência) ---------------
    numero_pontos_espera  = 250;   % resolução da fase de espera
    numero_pontos_transf  = 400;   % resolução do arco de transferência

    tempo_vetor_espera = linspace(0, tempo_espera, numero_pontos_espera); % [s]
    tempo_vetor_transf_local = linspace(0, tempo_transf, numero_pontos_transf); % [s]
    tempo_vetor_transf = tempo_espera + tempo_vetor_transf_local; % [s] já no tempo absoluto

    % --------- longitudes ao longo do tempo ------------------------------
    % alvo: circular todo o tempo
    longitude_target_espera = wrapTo2Pi(longitude_target_inicial + n_target_propagado * tempo_vetor_espera);
    longitude_target_transf = wrapTo2Pi(longitude_target_burn1     + n_target_propagado * tempo_vetor_transf_local);

    % perseguidor:
    % - na espera permanece circular em r_inicial
    longitude_chaser_espera = wrapTo2Pi(longitude_chaser_inicial + n_chaser_propagado * tempo_vetor_espera);

    % - na transferência varre pi rad (Hohmann coplanar, perigeu->apogeu)
    anomalia_transf_vetor = linspace(0, pi, numero_pontos_transf); % [rad]
    longitude_chaser_transf = wrapTo2Pi(longitude_chaser_burn1 + anomalia_transf_vetor);

    % --------- raios ao longo do tempo -----------------------------------
    % alvo tem raio constante
    raio_target_espera = raio_target * ones(size(tempo_vetor_espera));
    raio_target_transf = raio_target * ones(size(tempo_vetor_transf_local));

    % perseguidor:
    raio_chaser_espera = raio_chaser_inicial * ones(size(tempo_vetor_espera));

    % raio na elipse de transferência (Hohmann)
    % excentricidade da elipse de transferência
    excentricidade_orbita_transferencia = (raio_chaser_final - raio_chaser_inicial) / ...
                                          (raio_chaser_final + raio_chaser_inicial);
    parametro_semi_latus_orbita_transferencia = orbita_transf * (1 - excentricidade_orbita_transferencia^2);

    raio_chaser_transf = parametro_semi_latus_orbita_transferencia ./ ...
                         (1 + excentricidade_orbita_transferencia * cos(anomalia_transf_vetor));

    % --------- ECI vetorizado (plano X-Y) --------------------------------
    % alvo
    eci_target_espera_raio = [raio_target_espera .* cos(longitude_target_espera);
                              raio_target_espera .* sin(longitude_target_espera);
                              zeros(size(tempo_vetor_espera))];

    eci_target_transf_raio = [raio_target_transf .* cos(longitude_target_transf);
                              raio_target_transf .* sin(longitude_target_transf);
                              zeros(size(tempo_vetor_transf_local))];

    % perseguidor
    eci_chaser_espera_raio = [raio_chaser_espera .* cos(longitude_chaser_espera);
                              raio_chaser_espera .* sin(longitude_chaser_espera);
                              zeros(size(tempo_vetor_espera))];

    eci_chaser_transf_raio = [raio_chaser_transf .* cos(longitude_chaser_transf);
                              raio_chaser_transf .* sin(longitude_chaser_transf);
                              zeros(size(tempo_vetor_transf_local))];

    % --------- concatenação para gráficos temporais -----------------------
    tempo_vetor_total = [tempo_vetor_espera, tempo_vetor_transf]; % [s]

    longitude_target_total = [longitude_target_espera, longitude_target_transf]; % [rad]
    longitude_chaser_total = [longitude_chaser_espera, longitude_chaser_transf]; % [rad]

    eci_target_total_raio = [eci_target_espera_raio, eci_target_transf_raio]; % 3xN
    eci_chaser_total_raio = [eci_chaser_espera_raio, eci_chaser_transf_raio]; % 3xN

    raio_target_total = [raio_target_espera, raio_target_transf];     % [m]
    raio_chaser_total = [raio_chaser_espera, raio_chaser_transf];     % [m]

    % distância relativa em ECI ao longo do tempo
    distancia_relativa_total = vecnorm(eci_chaser_total_raio - eci_target_total_raio, 2, 1); % [m]

% ======================================================================
%% FIGURA 1) Geometria ECI 2D (COMPLETA)
% ======================================================================
figure('Name','Aproximação de longa distância (ECI): Transferência de órbita');
hold on; grid on; axis equal;

% fator para converter m -> km no gráfico
fator_escala_km = 1e-3;

% ---------------- Terra (círculo preenchido azul claro) -----------------
angulo_circulo = linspace(0, 2*pi, 500);
x_terra = raio_Terra*cos(angulo_circulo);
y_terra = raio_Terra*sin(angulo_circulo);

terra_handle = fill(x_terra*fator_escala_km, y_terra*fator_escala_km, ...
    [0.70 0.85 1.00], ...                         % azul claro
    'EdgeColor','k', 'LineWidth', 1.2);           % contorno preto

% ---------------- Órbita do ALVO (vermelha) -----------------------------
orbita_alvo_handle = plot(raio_target*cos(angulo_circulo)*fator_escala_km, ...
                          raio_target*sin(angulo_circulo)*fator_escala_km, ...
                          'Color',[0.85 0.10 0.10], 'LineWidth', 1.2);

% ---------------- Órbita inicial do PERSEGUIDOR (azul) ------------------
orbita_perseguidor_handle = plot(raio_chaser_inicial*cos(angulo_circulo)*fator_escala_km, ...
                                 raio_chaser_inicial*sin(angulo_circulo)*fator_escala_km, ...
                                 'Color',[0.00 0.45 0.85], 'LineWidth', 1.2);

% ---------------- Arco elíptico de transferência (perseguidor) ----------
arco_transferencia_handle = plot(eci_chaser_transf_raio(1,:)*fator_escala_km, ...
                                 eci_chaser_transf_raio(2,:)*fator_escala_km, ...
                                 'Color',[0.2 0.2 0.2], 'LineWidth', 2.0);

% ---------------- Pontos iniciais (maiores) -----------------------------
inicio_perseguidor_handle = plot(eci_chaser_inicial_raio(1)*fator_escala_km, ...
                                eci_chaser_inicial_raio(2)*fator_escala_km, ...
                                'o', 'MarkerSize', 12, 'LineWidth', 2.2, ...
                                'Color',[0.00 0.45 0.85], ...
                                'MarkerFaceColor',[0.00 0.45 0.85]);   % perseguidor azul

inicio_alvo_handle = plot(eci_target_inicial_raio(1)*fator_escala_km, ...
                          eci_target_inicial_raio(2)*fator_escala_km, ...
                          'o', 'MarkerSize', 12, 'LineWidth', 2.2, ...
                          'Color',[0.85 0.10 0.10], ...
                          'MarkerFaceColor',[0.85 0.10 0.10]);         % alvo vermelho

% ---------------- Posições no instante da QUEIMA 1 ----------------------
eci_chaser_burn1_raio_para_grafico = raio_chaser_inicial * ...
    [cos(longitude_chaser_burn1); sin(longitude_chaser_burn1); 0];

eci_target_burn1_raio_para_grafico = raio_target * ...
    [cos(longitude_target_burn1); sin(longitude_target_burn1); 0];

burn1_perseguidor_handle = plot(eci_chaser_burn1_raio_para_grafico(1)*fator_escala_km, ...
                                eci_chaser_burn1_raio_para_grafico(2)*fator_escala_km, ...
                                '^', 'MarkerSize', 11, 'LineWidth', 2.2, ...
                                'Color',[0.00 0.45 0.85], ...
                                'MarkerFaceColor',[0.00 0.45 0.85]);   % perseguidor em Δv1 (azul)

burn1_alvo_handle = plot(eci_target_burn1_raio_para_grafico(1)*fator_escala_km, ...
                         eci_target_burn1_raio_para_grafico(2)*fator_escala_km, ...
                         'd', 'MarkerSize', 10, 'LineWidth', 2.2, ...
                         'Color',[0.85 0.10 0.10], ...
                         'MarkerFaceColor',[0.85 0.10 0.10]);          % alvo no instante Δv1 (vermelho)

% ---------------- Setas só nos instantes de Δv do perseguidor -----------
% escala visual comum para as duas setas (maiores e mesma grossura)
escala_seta_deltav = 0.25 * raio_Terra / max(abs(deltav1), abs(deltav2));

% direção tangencial no ponto Δv1
direcao_tangencial_chaser_burn1 = [-sin(longitude_chaser_burn1); cos(longitude_chaser_burn1); 0];
direcao_tangencial_chaser_burn1 = direcao_tangencial_chaser_burn1 / norm(direcao_tangencial_chaser_burn1);

% seta Δv1 (PRETA)
seta_deltav1 = quiver(eci_chaser_burn1_raio_para_grafico(1)*fator_escala_km, ...
                      eci_chaser_burn1_raio_para_grafico(2)*fator_escala_km, ...
                      (deltav1 * direcao_tangencial_chaser_burn1(1) * escala_seta_deltav)*fator_escala_km, ...
                      (deltav1 * direcao_tangencial_chaser_burn1(2) * escala_seta_deltav)*fator_escala_km, ...
                      'Color','k', 'LineWidth', 2.0, 'MaxHeadSize', 2.4);

% direção tangencial no ponto Δv2 (apogeu / circularização)
direcao_tangencial_chaser_burn2 = [-sin(longitude_chaser_burn2); cos(longitude_chaser_burn2); 0];
direcao_tangencial_chaser_burn2 = direcao_tangencial_chaser_burn2 / norm(direcao_tangencial_chaser_burn2);

% seta Δv2 (AZUL)
seta_deltav2 = quiver(eci_chaser_final_raio(1)*fator_escala_km, ...
                      eci_chaser_final_raio(2)*fator_escala_km, ...
                      (deltav2 * direcao_tangencial_chaser_burn2(1) * escala_seta_deltav)*fator_escala_km, ...
                      (deltav2 * direcao_tangencial_chaser_burn2(2) * escala_seta_deltav)*fator_escala_km, ...
                      'Color',[0.00 0.45 0.85], 'LineWidth', 2.0, 'MaxHeadSize', 2.4);

% ---------------- Pontos finais (instante da Δv2) ------------------------
final_perseguidor_handle = plot(eci_chaser_final_raio(1)*fator_escala_km, ...
                                eci_chaser_final_raio(2)*fator_escala_km, ...
                                's', 'MarkerSize', 13, 'LineWidth', 2.6, ...
                                'Color',[0.00 0.45 0.85], ...
                                'MarkerFaceColor',[0.00 0.45 0.85]);   % final perseguidor azul

final_alvo_handle = plot(eci_target_final_raio(1)*fator_escala_km, ...
                         eci_target_final_raio(2)*fator_escala_km, ...
                         's', 'MarkerSize', 13, 'LineWidth', 2.6, ...
                         'Color',[0.85 0.10 0.10], ...
                         'MarkerFaceColor',[0.85 0.10 0.10]);          % final alvo vermelho

% ---------------- Legenda espaçada (sem sobreposição) -------------------
xlabel('ECI X [km]');
ylabel('ECI Y [km]');

% linhas invisíveis para espaçar a legenda
espaco_legend1 = plot(nan, nan, 'w');
espaco_legend2 = plot(nan, nan, 'w');

legenda = legend([ ...
    terra_handle, ...
    espaco_legend1, ...
    orbita_alvo_handle, ...
    espaco_legend2, ...
    orbita_perseguidor_handle, ...
    espaco_legend1, ...
    arco_transferencia_handle, ...
    espaco_legend2, ...
    inicio_perseguidor_handle, ...
    espaco_legend1, ...
    inicio_alvo_handle, ...
    espaco_legend2, ...
    burn1_perseguidor_handle, ...
    espaco_legend1, ...
    burn1_alvo_handle, ...
    espaco_legend2, ...
    seta_deltav1, ...
    espaco_legend1, ...
    seta_deltav2, ...
    espaco_legend2, ...
    final_perseguidor_handle, ...
    espaco_legend1, ...
    final_alvo_handle], ...
    { ...
    'Terra', ...
    ' ', ...
    'Órbita alvo (vermelha)', ...
    ' ', ...
    'Órbita inicial perseguidor (azul)', ...
    ' ', ...
    'Arco transferência perseguidor', ...
    ' ', ...
    'Início perseguidor', ...
    ' ', ...
    'Início alvo', ...
    ' ', ...
    'Perseguidor no instante Δv1', ...
    ' ', ...
    'Alvo no instante Δv1', ...
    ' ', ...
    'Instante Δv1 (preta)', ...
    ' ', ...
    'Instante Δv2 (azul)', ...
    ' ', ...
    'Final perseguidor (Δv2)', ...
    ' ', ...
    'Final alvo (Δv2)'}, ...
    'Location','bestoutside');


set(legenda, 'FontSize', 18, 'ItemTokenSize', [22 12], 'Interpreter','none');

% title('Aproximação de longa distância (ECI): Transferência de órbita');

%% FIGURA 2) Longitude (ECI) vs tempo  (0 no meio, TEMPO EM SEGUNDOS)
% ======================================================================
figure('Name','Aproximação de longa distância (ECI): Longitude ao longo do tempo');
hold on; grid on;

plot(tempo_vetor_total, ...
     rad2deg(wrapToPi(longitude_chaser_total - longitude_target_inicial)), ...
     'LineWidth', 1.6, 'Color','blue');

plot(tempo_vetor_total, ...
     rad2deg(wrapToPi(longitude_target_total - longitude_target_inicial)), ...
     'LineWidth', 1.6, 'Color','red');

yline(0,'-k','0 deg');

xline(tempo_espera, '--k', 'Queima 1', 'LabelVerticalAlignment','bottom');
xline(tempo_burn2,  '--k', 'Queima 2', 'LabelVerticalAlignment','bottom');

ylim([-180 180]);
yticks(-180:60:180);

xlabel('Tempo [s]');
ylabel('Longitude inercial relativa ao alvo inicial [deg]');
legend({'Perseguidor','Alvo'}, 'Location','eastoutside');
% title('Aproximação de longa distância (ECI): Longitude ao longo do tempo');

%% FIGURA 3) Raio orbital vs tempo (TEMPO EM SEGUNDOS)
% ======================================================================
figure('Name','Aproximação de longa distância (ECI): Raio orbital ao longo do tempo');
hold on; grid on;

plot(tempo_vetor_total, raio_chaser_total/1000, 'LineWidth', 1.6, 'Color','blue');
plot(tempo_vetor_total, raio_target_total/1000, 'LineWidth', 1.6, 'Color','red');

xline(tempo_espera, '--k', 'Queima 1', 'LabelVerticalAlignment','bottom');
xline(tempo_burn2,  '--k', 'Queima 2', 'LabelVerticalAlignment','bottom');

xlabel('Tempo [s]');
ylabel('Raio orbital [km]');
legend({'Perseguidor','Alvo'}, 'Location','eastoutside');
% title('Aproximação de longa distância (ECI): Raio orbital ao longo do tempo');

%% FIGURA 4) Distância relativa em ECI vs tempo (TEMPO EM SEGUNDOS)
% ======================================================================
figure('Name','Aproximação de longa distância (ECI): Distância relativa ao longo do tempo');
grid on; hold on;

plot(tempo_vetor_total, distancia_relativa_total/1000, 'LineWidth', 1.8, 'Color','black');

xline(tempo_espera, '--k', 'Queima 1', 'LabelVerticalAlignment','bottom');
xline(tempo_burn2,  '--k', 'Queima 2', 'LabelVerticalAlignment','bottom');

xlabel('Tempo [s]');
ylabel('Distância relativa [km]');
% title('Aproximação de longa distância (ECI): Distância relativa ao longo do tempo');
end  

% ============================ Utilitários ================================
function [rECI, vECI] = oe2eci(mu, a, e, inc, RAAN, argp, nu)

    p = a*(1 - e^2);
    r = p/(1 + e*cos(nu));
    % PQW
    r_pf = [r*cos(nu); r*sin(nu); 0];
    v_pf = sqrt(mu/p) * [-sin(nu); e + cos(nu); 0];

    % Matriz de rotação PQW->ECI
    cO = cos(RAAN); sO = sin(RAAN);
    ci = cos(inc);  si = sin(inc);
    cw = cos(argp); sw = sin(argp);

    C = [ cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,  sO*si;
          sO*cw + cO*sw*ci,  -sO*sw + cO*cw*ci, -cO*si;
          sw*si,              cw*si,             ci    ];

    rECI = C * r_pf;
    vECI = C * v_pf;
end

function ang = wrapTo2Pi(ang)
    ang = mod(ang, 2*pi);
end

function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function s = fmtHMS(t)
    h = floor(t/3600);
    m = floor(mod(t,3600)/60);
    ssec = floor(mod(t,60));
    s = sprintf('%02dh:%02dm:%02ds', h, m, ssec);
end

function s = fstr(x)
    if isnan(x), s = '—'; else, s = sprintf('%.2f%%',100*x); end
end

function R = R3(a), ca=cos(a); sa=sin(a); R=[ ca -sa 0; sa ca 0; 0 0 1]; end
function R = R1(a), ca=cos(a); sa=sin(a); R=[ 1 0 0; 0 ca -sa; 0 sa ca]; end

function oe = rv2oe(mu, rECI, vECI)
% rECI, vECI: 3x1 em ECI (SI)

rECI = rECI(:);
vECI = vECI(:);

R = norm(rECI);
V = norm(vECI);

hvec = cross(rECI, vECI);
h = norm(hvec);

evec = (1/mu)*((V^2 - mu/R)*rECI - dot(rECI,vECI)*vECI);
e = norm(evec);

inc = acos(hvec(3)/h);

k = [0;0;1];
nvec = cross(k, hvec);
n = norm(nvec);

if n < 1e-12
    RAAN = 0;
else
    RAAN = atan2(nvec(2), nvec(1));
    RAAN = mod(RAAN, 2*pi);
end

if n < 1e-12 || e < 1e-12
    argp = 0;
else
    argp = atan2( dot(cross(nvec, evec), hvec)/h, dot(nvec, evec) );
    argp = mod(argp, 2*pi);
end

if e < 1e-12
    nu = atan2( dot(cross(nvec, rECI), hvec)/h, dot(nvec, rECI) );
else
    nu = atan2( dot(cross(evec, rECI), hvec)/h, dot(evec, rECI) );
end
nu = mod(nu, 2*pi);

a = 1 / (2/R - V^2/mu);
p = h^2/mu;

oe = struct('a',a,'e',e,'inc',inc,'RAAN',RAAN,'argp',argp,'nu',nu,'h',h,'p',p);
end

