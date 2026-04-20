function resultado = PropagarTH_DoisCorpos()
% PropagarTH_DoisCorpos.m
% - Usa as mesmas regras de fase do TransferenciaOrbital
% - Aplica Δv1 e Δv2 (Hohmann) no perseguidor
% - Propaga alvo e perseguidor pelo problema de dois corpos
% - Ao final (t_drift), imprime posição inicial e final (ECI)

    clc;

    % ================== 1) Constantes e condições iniciais ==================
    mu         = Constantes.mu_Terra;      % [m^3/s^2]
    raio_Terra = Constantes.raio_Terra;    % [m]

    % Longitudes inerciais iniciais (graus -> rad)
    lon_c0 = deg2rad(Constantes.longitude_inercial_chaser_inicial);   % [rad]
    lon_t0 = deg2rad(Constantes.longitude_inercial_target_inicial);   % [rad]

    % Raios orbitais (circulares, coplanares)
    raio_chaser_inicial = raio_Terra + Constantes.altitude_chaser_inicial + ...
                          Constantes.ECI_erro_injecao_chaser_radial;   % [m]
    raio_target         = raio_Terra + Constantes.altitude_target_inicial; % [m]

    % Offsets desejados após TH (LVLH do alvo)
    offset_radial     = Constantes.delta_radial_TH;      % [m]
    offset_alongtrack = Constantes.delta_alongtrack_TH;  % [m]
    offset_crosstrack = Constantes.delta_crosstrack_TH;  % [m] (não entra na lógica de fase)

    % Raio final desejado do perseguidor após TH
    raio_chaser_final = raio_target + offset_radial;     % [m]

    if raio_chaser_final <= raio_chaser_inicial
        error('Assume transfer^encia de orbita inferior -> superior. r_final deve ser > r_chaser0.');
    end

    % Elementos orbitais iniciais (circulares, equatoriais)
    e   = 0;
    inc = 0;
    RAAN = 0;
    argp = 0;

    a_t0 = raio_target;
    a_c0 = raio_chaser_inicial;

    nu_t0 = lon_t0;     % anomalia verdadeira = longitude (orbita circular coplanar)
    nu_c0 = lon_c0;

    % Estados ECI iniciais via elementos orbitais
    [r_t0, v_t0] = oe2eci(mu, a_t0, e, inc, RAAN, argp, nu_t0);
    [r_c0, v_c0] = oe2eci(mu, a_c0, e, inc, RAAN, argp, nu_c0);

    % ================== 2) Cinemática e Δv da Hohmann ======================
    n_chaser_inicial = sqrt(mu / raio_chaser_inicial^3); % [rad/s]
    n_target         = sqrt(mu / raio_target^3);         % [rad/s]

    % Órbita de transferência (semi-eixo maior)
    a_transf   = 0.5 * (raio_chaser_inicial + raio_chaser_final); % [m]
    tempo_transf = pi * sqrt(a_transf^3 / mu);                    % [s] (meia-órbita)

    % Velocidades circulares
    v_circ_c0 = sqrt(mu / raio_chaser_inicial); % [m/s]
    v_circ_cf = sqrt(mu / raio_chaser_final);   % [m/s]

    % Velocidades na elipse de transferência (perigeu e apogeu)
    v_peri_transf = sqrt( 2*mu*raio_chaser_final / ...
                         (raio_chaser_inicial * (raio_chaser_inicial + raio_chaser_final)) );
    v_apo_transf  = sqrt( 2*mu*raio_chaser_inicial / ...
                         (raio_chaser_final * (raio_chaser_inicial + raio_chaser_final)) );

    % Δv1 e Δv2
    deltav1 = v_peri_transf - v_circ_c0;  % queima 1 (perigeu da TH)
    deltav2 = v_circ_cf   - v_apo_transf; % queima 2 (apogeu da TH)

    % ================== 3) Lógica de fase (igual ao código anterior) =======
    % Fase desejada ao final da TH (Δv2)
    phi_desejado = - offset_alongtrack / raio_target;   % [rad]

    % Fase inicial (0..2π)
    phi_inicial = wrapTo2Pi(lon_t0 - lon_c0);           % [rad]

    denom = (n_target - n_chaser_inicial);              % [rad/s]
    if abs(denom) < 1e-12
        error('n_t e n_c0 quase iguais – condicao de fase mal condicionada.');
    end

    tempo_espera_req = (phi_desejado - phi_inicial - (n_target*tempo_transf - pi)) / denom; % [s]

    % Período sinódico e normalização do tempo de espera
    T_syn = 2*pi / abs(denom);      % [s]
    tempo_espera = tempo_espera_req;
    if tempo_espera < 0
        k = ceil(-tempo_espera / T_syn);
        tempo_espera = tempo_espera + k*T_syn;
    end

    % Histereses (mesmas ideias do código original)
    tempo_espera_min = 5;           % [s]
    fase_tol_deg     = 1;
    fase_tol_rad     = deg2rad(fase_tol_deg);

    % Se a manobra começasse agora (tempo_espera = 0)
    lon_t_se_b2 = wrapTo2Pi(lon_t0 + n_target*tempo_transf);
    lon_c_se_b1 = wrapTo2Pi(lon_c0 + pi);  % chaser varre π na TH
    phi_se_agora = wrapToPi(lon_t_se_b2 - lon_c_se_b1);

    pulou = false;
    if tempo_espera < tempo_espera_min
        tempo_espera = tempo_espera + T_syn;
        pulou = true;
    end
    if ~pulou && abs(wrapToPi(phi_se_agora - phi_desejado)) <= fase_tol_rad
        tempo_espera = tempo_espera + T_syn;
        pulou = true;
    end

    % Instantes chave
    tempo_burn1 = tempo_espera;
    tempo_burn2 = tempo_espera + tempo_transf;

    % Drift pós-TH
    dt_drift = Constantes.tempo_drift;  % [s]
    t_drift  = tempo_burn2 + dt_drift;  % instante final da simulação

    % ================== 4) Propagação (problema de dois corpos) ============
    % Integração em 3 fases:
    %  - Fase 1: [0, tempo_espera]    -> órbitas circulares iniciais
    %  - Fase 2: [tempo_espera, tempo_burn2] (TH no perseguidor)
    %  - Fase 3: [tempo_burn2, t_drift]       -> órbitas circulares finais

    % Estado combinado: y = [r_t; v_t; r_c; v_c]
    y0 = [r_t0; v_t0; r_c0; v_c0];

    opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);

    % ---------- Fase 1: espera até Δv1 ----------
    if tempo_espera > 0
        [t1, y1] = ode45(@(t,y) doisCorposDuplo(t,y,mu), [0 tempo_espera], y0, opts);
        y_b1 = y1(end,:).';
    else
        % sem espera
        t1  = 0;
        y_b1 = y0;
    end

    r_t_b1 = y_b1(1:3);
    v_t_b1 = y_b1(4:6);
    r_c_b1 = y_b1(7:9);
    v_c_b1 = y_b1(10:12);

    % Queima 1: Δv1 aplicado ao perseguidor na direção da velocidade
    t_hat1 = v_c_b1 / norm(v_c_b1);
    v_c_b1p = v_c_b1 + deltav1 * t_hat1;    % pós-queima 1

    y2_0 = [r_t_b1; v_t_b1; r_c_b1; v_c_b1p];

    % ---------- Fase 2: órbita de transferência até Δv2 ----------
    [t2, y2] = ode45(@(t,y) doisCorposDuplo(t,y,mu), ...
                     [0 tempo_transf], y2_0, opts);
    y_b2 = y2(end,:).';

    r_t_b2 = y_b2(1:3);
    v_t_b2 = y_b2(4:6);
    r_c_b2 = y_b2(7:9);
    v_c_b2 = y_b2(10:12);

    % Queima 2: Δv2 aplicado ao perseguidor na direção da velocidade
    t_hat2 = v_c_b2 / norm(v_c_b2);
    v_c_b2p = v_c_b2 + deltav2 * t_hat2;    % pós-queima 2

    y3_0 = [r_t_b2; v_t_b2; r_c_b2; v_c_b2p];

    % ---------- Fase 3: drift pós-TH até t_drift ----------
    if dt_drift > 0
        [t3, y3] = ode45(@(t,y) doisCorposDuplo(t,y,mu), ...
                         [0 dt_drift], y3_0, opts);
        y_f = y3(end,:).';
    else
        t3  = 0;
        y3  = y3_0.';
        y_f = y3_0;
    end

    r_t_f = y_f(1:3);
    v_t_f = y_f(4:6);
    r_c_f = y_f(7:9);
    v_c_f = y_f(10:12);

    % ================== 5) Impressão no Command Window =====================
    fprintf('\n=========== PROPAGACAO 2-CORPOS COM TH ===========\n');
    fprintf('tempo_espera      = %.2f s\n', tempo_espera);
    fprintf('tempo_transf      = %.2f s\n', tempo_transf);
    fprintf('dt_drift          = %.2f s\n', dt_drift);
    fprintf('t_drift (final)   = %.2f s\n', t_drift);
    fprintf('--------------------------------------------------\n');
    fprintf('Δv1 = %.3f m/s\n', deltav1);
    fprintf('Δv2 = %.3f m/s\n', deltav2);
    fprintf('Δv_total = %.3f m/s\n', abs(deltav1)+abs(deltav2));
    fprintf('--------------------------------------------------\n');

    fprintf('ALVO (TARGET)\n');
    fprintf('r_t0 (ECI, m)  = [%.3f  %.3f  %.3f]\n', r_t0(1), r_t0(2), r_t0(3));
    fprintf('v_t0 (ECI, m/s)= [%.6f  %.6f  %.6f]\n', v_t0(1), v_t0(2), v_t0(3));
    fprintf('r_tf (ECI, m)  = [%.3f  %.3f  %.3f]\n', r_t_f(1), r_t_f(2), r_t_f(3));
    fprintf('v_tf (ECI, m/s)= [%.6f  %.6f  %.6f]\n', v_t_f(1), v_t_f(2), v_t_f(3));
    fprintf('--------------------------------------------------\n');

    fprintf('PERSEGUIDOR (CHASER)\n');
    fprintf('r_c0 (ECI, m)  = [%.3f  %.3f  %.3f]\n', r_c0(1), r_c0(2), r_c0(3));
    fprintf('v_c0 (ECI, m/s)= [%.6f  %.6f  %.6f]\n', v_c0(1), v_c0(2), v_c0(3));
    fprintf('r_cf (ECI, m)  = [%.3f  %.3f  %.3f]\n', r_c_f(1), r_c_f(2), r_c_f(3));
    fprintf('v_cf (ECI, m/s)= [%.6f  %.6f  %.6f]\n', v_c_f(1), v_c_f(2), v_c_f(3));
    fprintf('==================================================\n\n');

    % ================== 6) Saída estruturada ===============================
    resultado = struct();
    resultado.tempo_espera   = tempo_espera;
    resultado.tempo_transf   = tempo_transf;
    resultado.dt_drift       = dt_drift;
    resultado.t_drift        = t_drift;
    resultado.deltav1        = deltav1;
    resultado.deltav2        = deltav2;
    resultado.raio_target    = raio_target;
    resultado.raio_c0        = raio_chaser_inicial;
    resultado.raio_cf        = raio_chaser_final;

    resultado.r_t0 = r_t0;
    resultado.v_t0 = v_t0;
    resultado.r_c0 = r_c0;
    resultado.v_c0 = v_c0;

    resultado.r_t_b1 = r_t_b1;
    resultado.v_t_b1 = v_t_b1;
    resultado.r_c_b1 = r_c_b1;
    resultado.v_c_b1 = v_c_b1;
    resultado.r_t_b2 = r_t_b2;
    resultado.v_t_b2 = v_t_b2;
    resultado.r_c_b2 = r_c_b2;
    resultado.v_c_b2 = v_c_b2;

    resultado.r_t_f = r_t_f;
    resultado.v_t_f = v_t_f;
    resultado.r_c_f = r_c_f;
    resultado.v_c_f = v_c_f;

end

% =====================================================================
% Dinâmica de dois corpos para alvo e perseguidor simultaneamente
% y = [r_t; v_t; r_c; v_c]
% =====================================================================
function dydt = doisCorposDuplo(~, y, mu)

    r_t = y(1:3);
    v_t = y(4:6);
    r_c = y(7:9);
    v_c = y(10:12);

    a_t = -mu * r_t / norm(r_t)^3;
    a_c = -mu * r_c / norm(r_c)^3;

    dydt = [v_t;
            a_t;
            v_c;
            a_c];
end

% =====================================================================
% Conversão de elementos orbitais clássicos para ECI
% (mesma forma do seu código original)
% =====================================================================
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

% =====================================================================
% Utilitários de ângulo (iguais à sua lógica anterior)
% =====================================================================
function ang = wrapTo2Pi(ang)
    ang = mod(ang, 2*pi);
end

function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
