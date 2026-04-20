%% 
function TransfOrbital = TransferenciaOrbital()
% --------------------------------------------------------------------------
% TransferenciaOrbital.m
% --------------------------------------------------------------------------
    clc; 
    % ======= Constantes e condições iniciais (SI) ==========================
    mu         = Constantes.mu_Terra;              % [m^3/s^2]
    raio_Terra = Constantes.raio_Terra;            % [m]

    % Longitudes inerciais iniciais (deg -> rad)
    longitude_chaser_inicial = deg2rad(Constantes.longitude_inercial_chaser_inicial);  % [rad]
    longitude_target_inicial = deg2rad(Constantes.longitude_inercial_target_inicial);  % [rad]

    % Raios orbitais (circulares, coplanares)
    raio_chaser_inicial = raio_Terra + Constantes.altitude_chaser_inicial + ...
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
    deltav_total = abs(deltav1) + abs(deltav2);                      % [m/s]

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
    eci_target_final_raio  = raio_target * [cos(longitude_target_burn2);  ...
                                            sin(longitude_target_burn2); 0];
    eci_target_final_veloc = sqrt(mu / raio_target) * [ -sin(longitude_target_burn2); ...
                                                         cos(longitude_target_burn2); 0];

    % Chaser (após circularização na órbita final desejada)
    eci_chaser_final_raio  = raio_chaser_final * [cos(longitude_chaser_burn2); ...
                                                  sin(longitude_chaser_burn2); 0];
    eci_chaser_final_veloc = sqrt(mu / raio_chaser_final) * [ -sin(longitude_chaser_burn2); ...
                                                               cos(longitude_chaser_burn2); 0];

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

    % ===== Fase após drift (15 min) =====
    phi_drift_target_menos_chaser = wrapToPi(theta_t_drift - theta_c_drift); % [rad]
    phi_drift_chaser_menos_target = -phi_drift_target_menos_chaser;         % [rad]

    % ===== Salvar longitudes no drift =====
    TransfOrbital.theta_target_drift_rad = theta_t_drift;
    TransfOrbital.theta_chaser_drift_rad = theta_c_drift;
   
    
    % ECI em t_drift
    eci_target_drift_r = raio_target * [cos(theta_t_drift);  sin(theta_t_drift);  0];
    eci_chaser_drift_r = raio_chaser_final * [cos(theta_c_drift); sin(theta_c_drift); 0];

    eci_target_drift_v = sqrt(mu/raio_target)       * [-sin(theta_t_drift);  cos(theta_t_drift);  0];
    eci_chaser_drift_v = sqrt(mu/raio_chaser_final) * [-sin(theta_c_drift);  cos(theta_c_drift);  0];

    % ECI em t_drift via COEs
    nu_t_drift = theta_t_drift;   % já calculado no seu bloco de drift
    nu_c_drift = theta_c_drift;
    
    a_t_drift = raio_target;        % circular
    a_c_drift = raio_chaser_final;  % circular
    
    [eci_target_drift_r, eci_target_drift_v] = oe2eci(mu, a_t_drift, e, inc, RAAN, argp, nu_t_drift);
    [eci_chaser_drift_r, eci_chaser_drift_v] = oe2eci(mu, a_c_drift, e, inc, RAAN, argp, nu_c_drift);

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

% ======= Pacote de saída ===========================
% Esta struct reúne todos os resultados calculados na etapa de
% transferência orbital, para uso posterior no workspace, Simulink
% ou scripts de análise/gráficos.
TransfOrbital = struct();
    
% =========================
% 1) Tempos principais da manobra
% =========================
% tempo_espera  : tempo aguardado antes do primeiro burn
% tempo_transf  : duração da transferência orbital
% tempo_burn2   : instante em que ocorre o segundo burn
TransfOrbital.tempo_espera             = tempo_espera;
TransfOrbital.tempo_transf             = tempo_transf;
TransfOrbital.tempo_burn2              = tempo_burn2;

% =========================
% 2) Delta-V da manobra
% =========================
% deltav1      : impulso do primeiro burn
% deltav2      : impulso do segundo burn
% deltav_total : soma total de delta-V da transferência
TransfOrbital.deltav1                  = deltav1;
TransfOrbital.deltav2                  = deltav2;
TransfOrbital.deltav_total             = deltav_total;

% =========================
% 3) Raios orbitais e fase inicial/final
% =========================
% raio_chaser_inicial : raio orbital inicial do chaser
% raio_target         : raio orbital do target
% raio_chaser_final   : raio final do chaser após a transferência
% phi_inicial         : defasagem angular inicial
% fase_desejada_phi   : fase angular desejada para encontro
% fase_alcancada_phi2 : fase efetivamente alcançada no burn 2
TransfOrbital.raio_chaser_inicial      = raio_chaser_inicial;
TransfOrbital.raio_target              = raio_target;
TransfOrbital.raio_chaser_final        = raio_chaser_final;
TransfOrbital.phi_inicial              = phi_inicial;
TransfOrbital.fase_desejada_phi        = phi_desejado;
TransfOrbital.fase_alcancada_phi2      = phi_final;

% =========================
% 4) Longitudes no burn 1
% =========================
% Guarda as longitudes do chaser e do target no instante do burn 1,
% já em graus, para facilitar inspeção.
TransfOrbital.longitude_chaser_burn1   = rad2deg(longitude_chaser_burn1);
TransfOrbital.longitude_target_burn1   = rad2deg(longitude_target_burn1);

% Calcula a fase relativa target - chaser no burn 1
phi_burn1 = wrapToPi(longitude_target_burn1 - longitude_chaser_burn1); % [rad]

% Salva novamente as longitudes em graus com nomes explícitos
TransfOrbital.longitude_chaser_burn1_deg = rad2deg(longitude_chaser_burn1);
TransfOrbital.longitude_target_burn1_deg = rad2deg(longitude_target_burn1);

% Salva a fase relativa no burn 1 em rad e graus
TransfOrbital.fase_no_burn1_phi_rad = phi_burn1;
TransfOrbital.fase_no_burn1_phi_deg = rad2deg(phi_burn1);

% =========================
% 5) Longitudes e fase relativa no burn 2
% =========================
% Longitudes absolutas do target e do chaser no segundo burn
TransfOrbital.longitude_target_burn2   = longitude_target_burn2;
TransfOrbital.longitude_chaser_burn2   = longitude_chaser_burn2;

% Fase relativa no burn 2:
% primeiro calcula target - chaser,
% depois salva também chaser - target
phi_burn2_target_menos_chaser = wrapToPi(longitude_target_burn2 - longitude_chaser_burn2);
phi_burn2_chaser_menos_target = -phi_burn2_target_menos_chaser;
    
TransfOrbital.fase_chaser_rel_target_burn2_rad = phi_burn2_chaser_menos_target;
TransfOrbital.fase_chaser_rel_target_burn2_deg = rad2deg(phi_burn2_chaser_menos_target);

% =========================
% 6) Estados ECI no início da missão
% =========================
% Vetores posição e velocidade inerciais do target e do chaser
% no instante inicial da simulação
TransfOrbital.eci_target_inicial_raio  = eci_target_inicial_raio;
TransfOrbital.eci_target_inicial_veloc = eci_target_inicial_veloc;
TransfOrbital.eci_chaser_inicial_raio  = eci_chaser_inicial_raio;
TransfOrbital.eci_chaser_inicial_veloc = eci_chaser_inicial_veloc;

% =========================
% 7) Estados ECI no instante do burn 2
% =========================
% Vetores posição e velocidade inerciais do target e do chaser
% no instante final da transferência
TransfOrbital.eci_target_final_raio  = eci_target_final_raio;
TransfOrbital.eci_target_final_veloc = eci_target_final_veloc;
TransfOrbital.eci_chaser_final_raio  = eci_chaser_final_raio;
TransfOrbital.eci_chaser_final_veloc = eci_chaser_final_veloc;
        
% =========================
% 8) Parâmetros orbitais auxiliares
% =========================
% T_syn         : período síncrono/orbital usado no cálculo
% t_espera_min  : passo mínimo de busca para o tempo de espera
% fase_tol_deg  : tolerância angular admitida
TransfOrbital.T_syn                    = T_syn;
TransfOrbital.t_espera_min             = tempo_espera_min;
TransfOrbital.fase_tol_deg             = fase_tol_deg;

% =========================
% 9) Resultado relativo em LVLH no burn 2
% =========================
% Comparação entre posição desejada e alcançada no referencial LVLH
TransfOrbital.alongtrack_desejado_m   = along_des_m;
TransfOrbital.alongtrack_alcancado_m  = along_fin_m;
TransfOrbital.radial_desejado_m       = radial_des_m;
TransfOrbital.radial_alcancado_m      = radial_fin_m;
TransfOrbital.crosstrack_desejado_m   = crosstrack_des_m;
TransfOrbital.crosstrack_alcancado_m  = crosstrack_fin_m;
    
% =========================
% 10) Erros e tolerâncias da solução
% =========================
% Guarda os erros finais da transferência e as tolerâncias adotadas
TransfOrbital.err_fase_rad            = erro_fase;
TransfOrbital.tol_fase_rad            = tol_fase;
TransfOrbital.err_along_m             = err_along;
TransfOrbital.tol_along_m             = tol_along;
TransfOrbital.err_radial_m            = err_radial;
TransfOrbital.tol_radial_m            = tol_radial;
TransfOrbital.err_crosstrack_m        = err_cross;
TransfOrbital.tol_crosstrack_m        = tol_cross;

% =========================
% 11) Estado relativo LVLH no instante do burn 2
% =========================
% Vetores posição e velocidade relativas no referencial LVLH
TransfOrbital.lvlh_rel_pos_b2 = r_rel_lvlh;   % [x;y;z] em metros
TransfOrbital.lvlh_rel_vel_b2 = v_rel_lvlh;   % [vx;vy;vz] em m/s

% Componentes escalares da posição relativa LVLH no burn 2
TransfOrbital.radial_b2     = radial_b2;
TransfOrbital.alongtrack_b2 = alongtrack_b2;
TransfOrbital.crosstrack_b2 = crosstrack_b2;

% Componentes escalares da velocidade relativa LVLH no burn 2
TransfOrbital.vradial_b2     = vradial_b2;
TransfOrbital.valongtrack_b2 = valongtrack_b2;
TransfOrbital.vcrosstrack_b2 = vcrosstrack_b2;

% =========================
% 12) Estado relativo após drift
% =========================
% Resultado relativo após o período de drift posterior à transferência
TransfOrbital.t_drift  = t_drift;
TransfOrbital.dt_drift = dt_drift;
TransfOrbital.lvlh_rel_pos_drift = r_rel_lvlh_s;
TransfOrbital.lvlh_rel_vel_drift = v_rel_lvlh_s;

% =========================
% 13) Fase relativa após drift
% =========================
% Ângulos absolutos de target e chaser e fase relativa correspondente
TransfOrbital.theta_target_drift_rad = theta_t_drift;
TransfOrbital.theta_chaser_drift_rad = theta_c_drift;
    
phi_drift = wrapToPi(theta_t_drift - theta_c_drift); % target - chaser
TransfOrbital.fase_drift_target_menos_chaser_rad = phi_drift;
TransfOrbital.fase_drift_target_menos_chaser_deg = rad2deg(phi_drift);

TransfOrbital.fase_drift_chaser_menos_target_rad = -phi_drift;
TransfOrbital.fase_drift_chaser_menos_target_deg = -rad2deg(phi_drift);

% Componentes da posição relativa após drift
TransfOrbital.radial_drift     = radial_drift;
TransfOrbital.alongtrack_drift = alongtrack_drift;
TransfOrbital.crosstrack_drift = crosstrack_drift;

% Componentes da velocidade relativa após drift
TransfOrbital.vradial_drift     = vradial_drift;
TransfOrbital.valongtrack_drift = valongtrack_drift;
TransfOrbital.vcrosstrack_drift = vcrosstrack_drift;

% =========================
% 14) Informação extra de tempo
% =========================
% Tempo total acumulado até o final do drift
TransfOrbital.tempo_total_ate_fim_drift = t_drift;

% =========================
% 15) Estado relativo inicial em ECI
% =========================
% Diferença inicial entre chaser e target no referencial inercial
r_rel_eci_0 = eci_chaser_inicial_raio - eci_target_inicial_raio;
v_rel_eci_0 = eci_chaser_inicial_veloc - eci_target_inicial_veloc;
dist_rel_inicio_missao  = norm(r_rel_eci_0);

TransfOrbital.r_rel_eci_0  = r_rel_eci_0;
TransfOrbital.v_rel_eci_0  = v_rel_eci_0;
TransfOrbital.dist_rel_inicio_missao = dist_rel_inicio_missao;

% Observação:
% este bloco nao gera graficos.
% Ele apenas organiza os resultados em uma struct.
% Os graficos, se existirem, devem estar em scripts separados, como os
% arquivos Graficos_*.m ou outros scripts de pos-processamento.

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