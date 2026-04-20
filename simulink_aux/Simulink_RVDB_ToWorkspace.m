%% Rodar o modelo
out = sim('Simulink_RVDB');

% tempo da simulação
t_RVDB = out.tout(:);   % [N x 1]

%% BLOCO CONTROLE ATITUDE PD
PD_torque_controle_attde        = ts_vec(out.PD_torque_controle_attde);
PD_torque_controle_compensado   = ts_vec(out.PD_torque_controle_compensado);
PD_ganho_proporcional           = ts_vec(out.PD_ganho_proporcional);
PD_ganho_derivativo             = ts_vec(out.PD_ganho_derivativo);

%% ATITUDE
attde_quaternion_target_referencia = ts_vec(out.attde_quaternion_target_referencia);  % [N x 4]
attde_quaternion_chaser            = ts_vec(out.attde_quaternion_chaser);             % [N x 4]
attd_angEuler_chaser               = ts_vec(out.attd_angEuler_chaser);                % [N x 3]
attd_quaternion_erroVetorial       = ts_vec(out.attd_quaternion_erroVetorial);        % [N x 4 ou 3]

%% DINÂMICA DE EULER (modo não-operacional)
dinEuler_acel_angular              = ts_vec(out.dinEuler_acel_angular);               % [N x 3]

%% DINÂMICA DE LAGRANGE (modo operacional, base + juntas)
dinLagrange_acel_angular_base_raw  = ts_vec(out.dinLagrange_acel_angular_base_raw);   % [N x 3]
dinLagrange_acel_angular_base      = ts_vec(out.dinLagrange_acel_angular_base);       % [N x 3]
dinLagrange_veloc_angular_base_raw = ts_vec(out.dinLagrange_veloc_angular_base_raw);  % [N x 3]
dinLagrange_veloc_angular_base     = ts_vec(out.dinLagrange_veloc_angular_base);      % [N x 3]

dinLagrange_acel_juntas_raw        = ts_vec(out.dinLagrange_acel_juntas_raw);         % [N x 3]
dinLagrange_acel_juntas            = ts_vec(out.dinLagrange_acel_juntas);             % [N x 3]
dinLagrange_veloc_juntas_raw       = ts_vec(out.dinLagrange_veloc_juntas_raw);        % [N x 3]
dinLagrange_veloc_juntas           = ts_vec(out.dinLagrange_veloc_juntas);            % [N x 3]
dinLagrange_posicao_juntas_raw     = ts_vec(out.dinLagrange_posicao_juntas_raw);      % [N x 3]
dinLagrange_posicao_juntas         = ts_vec(out.dinLagrange_posicao_juntas);          % [N x 3]

dinLagrange_veloc_juntas_fulltime   = ts_vec(out.dinLagrange_veloc_juntas_fulltime);
dinLagrange_posicao_juntas_fulltime = ts_vec(out.dinLagrange_posicao_juntas_fulltime);

%% Transição aceleração Euler - Lagrange

dinEulerLagrange_veloc_angular_viaIntegral = ts_vec(out.dinEulerLagrange_veloc_angular_viaIntegral);
dinEulerLagrange_acel_angular_viaIntegral  = ts_vec(out.dinEulerLagrange_acel_angular_viaIntegral);

%% OUTROS SINAIS DINÂMICOS
matriz_M_inercia_total            = out.matriz_M_inercia_total.signals.values;  % matriz, mantida como está
matriz_C_Coriolis                 = out.matriz_C_Coriolis.signals.values;       % idem
torque_reacao_base                = ts_vec(out.torque_reacao_base);            % [N x 3]

rampa_suave_alpha                          = ts_vec(out.rampa_suave_alpha);
vetor_estados_19x1                         = ts_vec(out.vetor_estados_19x1);   % [N x 19]

%% JACOBIANAS (mantidas 3D)
jacobiana_linear  = out.jacobiana_linear.signals.values;   % [3 x 3 x N]
jacobiana_angular = out.jacobiana_angular.signals.values;  % [3 x 3 x N]
jacob_porElo      = out.jacob_porElo.signals.values;       % [3 x 3 x N x elo]
jacob_deriv_porElo= out.jacob_deriv_porElo.signals.values; % idem

%% ENERGIAS
energia_cinetica_base  = ts_vec(out.energia_cinetica_base);  % [N x 1]
energia_cinetica_elos  = ts_vec(out.energia_cinetica_elos);  % [N x 1]
energia_cinetica_total = ts_vec(out.energia_cinetica_total); % [N x 1]

%% MODO DE OPERAÇÃO: extrair instante de comutação e gerar degrau ideal

% tempo próprio do modo
t_modo   = out.modo_operacao.time(:);    % [Nm x 1]
modo_raw = ts_vec(out.modo_operacao);    % [Nm x 1], valores 0 ou 1

% índice da primeira vez que o modo virou 1
idx_switch = find(modo_raw >= 0.5, 1, 'first');

if isempty(idx_switch)
    % nunca comutou: fica tudo zero
    t_switch = Inf;
else
    t_switch = t_modo(idx_switch);   % instante de comutação em segundos
end

% constrói degrau ideal no tempo global da simulação (0–1200 s)
modo_operacao = double(t_RVDB >= t_switch);   % [N_RVDB x 1]

%% VELOCIDADES ANGULARES ORBITAIS E LVLH
veloc_ang_corpo_wrt_ECI         = ts_vec(out.veloc_ang_corpo_wrt_ECI);        % [N x 3]
veloc_ang_corpoLVLH_wrtCorpo    = ts_vec(out.veloc_ang_corpoLVLH_wrtCorpo);   % [N x 3]

%% MOMENTO ANGULAR
momentum_angular_total_base     = ts_vec(out.momentum_angular_total_base);    % [N x 3]
momentum_angular_base_B         = ts_vec(out.momentum_angular_base_B);        % [N x 3]
momentum_angular_elos_B         = ts_vec(out.momentum_angular_elos_B);        % [N x 3]

%% POSIÇÃO DA FERRAMENTA, ERROS E ALVO
eef_posicao_atual_frame0        = ts_vec(out.eef_posicao_atual_frame0);       % [N x 3]
eef_erro_LVLH                   = ts_vec(out.eef_erro_LVLH);                  % [N x 3]
eef_posicao_LVLH                = ts_vec(out.eef_posicao_LVLH);               % [N x 3]
posicao_alvo_wrtCoM_BaseFisica  = ts_vec(out.posicao_alvo_wrtCoM_BaseFisica); % [N x 3]
posicao_alvo_wrtframe0_BaseFisica = ts_vec(out.posicao_alvo_wrtframe0_BaseFisica); % [N x 3]

erro_posicao_desejada_ferramenta       = ts_vec(out.erro_posicao_desejada_ferramenta);       % [N x 3]
erro_posicao_desejada_ferramenta_norma = ts_vec(out.erro_posicao_desejada_ferramenta_norma); % [N x 1]

%% IK (cinemática inversa)
ik_veloc_juntas                 = ts_vec(out.ik_veloc_juntas);               % [N x 3]
ik_posicao_juntas               = ts_vec(out.ik_posicao_juntas);             % [N x 3]

%% QUATERNION LVLH -> FRAME0 E RADIAL
quaternion_LVLH_frame0          = ts_vec(out.quaternion_LVLH_frame0);       % [N x 4]
radial0                         = ts_vec(out.radial0);                      % [N x 3]

%% ELOS (verificação geométrica) – mantidos 3D
elos_posicao                    = out.elos_posicao.signals.values;
elos_direcao                    = out.elos_direcao.signals.values;
elos_eixosZ                     = out.elos_eixosZ.signals.values;
elos_erro_ortho                 = out.elos_erro_ortho.signals.values;
elos_detR                       = out.elos_detR.signals.values;
elos_erro_det                   = out.elos_erro_det.signals.values;

%% TORQUES NAS JUNTAS
juntas_torque_controle_raw      = ts_vec(out.juntas_torque_controle_raw);    % [N x 3]
juntas_torque_controle          = ts_vec(out.juntas_torque_controle);        % [N x 3]

%% ERROS

dsu_alinhamento_pos_ref_f0_saida = ts_vec(out.alinhamento_pos_ref_f0_saida);
dsu_alinhamento_norma_erro_pos_ferramenta_m = ts_vec(out.alinhamento_norma_erro_pos_ferramenta_m);
dsu_alinhamento_norma_erro_pos_ferramenta_cm = ts_vec(out.alinhamento_norma_erro_pos_ferramenta_cm);
dsu_alinhamento_erro_pos_ferramenta_m = ts_vec(out.alinhamento_erro_pos_ferramenta_m);
dsu_alinhamento_erro_pos_ferramenta_cm = ts_vec(out.alinhamento_erro_pos_ferramenta_cm);

%% ALINHAMENTO

dsu_alinhamento_cond_pos_ok = ts_vec(out.alinhamento_cond_pos_ok);
dsu_alinhamento_cond_angulo_ok = ts_vec(out.alinhamento_cond_angulo_ok);
dsu_alinhamento_estado_sequenciador = ts_vec(out.alinhamento_estado_sequenciador);

%% Salvar tudo em .mat
% 1) SALVAR (trajetoria_RVDB.mat):

save('trajetoria_RVDB.mat', ...
    't_RVDB', ...
    'tempo_final', ...  
    'PD_torque_controle_attde', ...
    'PD_torque_controle_compensado', ...
    'PD_ganho_proporcional', ...
    'PD_ganho_derivativo', ...
    'attde_quaternion_target_referencia', ...
    'attde_quaternion_chaser', ...
    'attd_angEuler_chaser', ...
    'attd_quaternion_erroVetorial', ...
    'dinEuler_acel_angular', ...
    'dinLagrange_acel_angular_base_raw', ...
    'dinLagrange_acel_angular_base', ...
    'dinLagrange_veloc_angular_base_raw', ...
    'dinLagrange_veloc_angular_base', ...
    'dinLagrange_acel_juntas_raw', ...
    'dinLagrange_acel_juntas', ...
    'dinLagrange_veloc_juntas_raw', ...
    'dinLagrange_veloc_juntas', ...
    'dinLagrange_posicao_juntas_raw', ...
    'dinLagrange_posicao_juntas', ...
    'dinLagrange_posicao_juntas_fulltime', ...
    'dinLagrange_veloc_juntas_fulltime', ...
    'matriz_M_inercia_total', ...
    'matriz_C_Coriolis', ...
    'torque_reacao_base', ...
    'modo_operacao', ...
    'dinEulerLagrange_veloc_angular_viaIntegral', ...
    'dinEulerLagrange_acel_angular_viaIntegral', ...
    'rampa_suave_alpha', ...
    'vetor_estados_19x1', ...
    'jacobiana_linear', ...
    'jacobiana_angular', ...
    'jacob_porElo', ...
    'jacob_deriv_porElo', ...
    'energia_cinetica_base', ...
    'energia_cinetica_elos', ...
    'energia_cinetica_total', ...
    'veloc_ang_corpo_wrt_ECI', ...
    'veloc_ang_corpoLVLH_wrtCorpo', ...
    'momentum_angular_total_base', ...
    'momentum_angular_base_B', ...
    'momentum_angular_elos_B', ...
    'eef_posicao_atual_frame0', ...
    'eef_erro_LVLH', ...
    'eef_posicao_LVLH', ...
    'posicao_alvo_wrtCoM_BaseFisica', ...
    'posicao_alvo_wrtframe0_BaseFisica', ...
    'erro_posicao_desejada_ferramenta', ...
    'erro_posicao_desejada_ferramenta_norma', ...
    'ik_veloc_juntas', ...
    'ik_posicao_juntas', ...
    'quaternion_LVLH_frame0', ...
    'radial0', ...
    'elos_posicao', ...
    'elos_direcao', ...
    'elos_eixosZ', ...
    'elos_erro_ortho', ...
    'elos_detR', ...
    'elos_erro_det', ...
    'juntas_torque_controle_raw', ...
    'juntas_torque_controle',...
    'dsu_alinhamento_pos_ref_f0_saida',...
    'dsu_alinhamento_cond_pos_ok',...
    'dsu_alinhamento_norma_erro_pos_ferramenta_m',...
    'dsu_alinhamento_norma_erro_pos_ferramenta_cm',...
    'dsu_alinhamento_erro_pos_ferramenta_m',...
    'dsu_alinhamento_erro_pos_ferramenta_cm',...
    'dsu_alinhamento_cond_pos_ok',...
    'dsu_alinhamento_cond_angulo_ok',...
    'dsu_alinhamento_estado_sequenciador');


%% ------------------------------------------------------------------------
%% Função auxiliar: converte struct-with-time em matriz [N x dim]
function M = ts_vec(sig)
    X = sig.signals.values;
    s = size(X);

    if numel(s) == 3 && s(2) == 1
        % Formato típico: [dim x 1 x N] -> [N x dim]
        dim = s(1);
        N   = s(3);
        M   = reshape(permute(X, [3 1 2]), [N dim]);

    elseif numel(s) == 2
        % Casos 2D: vetor ou matriz dim x N
        if (s(1) == 1 || s(2) == 1)
            % Vetor 1D (1xN ou Nx1)
            if s(1) == 1 && s(2) > 1
                % 1xN -> N x 1
                M = X.';
            else
                % Nx1 já está no formato N x 1
                M = X;
            end
        else
            % Matriz genérica 2D
            % Se a primeira dimensão é pequena (<=10) e menor que a segunda,
            % interpreta como [dim x N] (canais x tempo) -> [N x dim]
            if s(1) <= 10 && s(1) < s(2)
                M = X.';      % [dim x N] -> [N x dim]
            else
                % Caso contrário, mantém como está (por segurança)
                M = X;
            end
        end

    else
        % Tensor (por exemplo jacobiana 3x3xN): mantém como está
        M = X;
    end
end
