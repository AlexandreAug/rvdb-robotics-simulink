% test_inverse_kinematics.m
% Teste em MATLAB do bloco Simulink_Inverse_Kinematics (DLS + null-space + filtro + integração).
% Salve como test_inverse_kinematics.m e execute no MATLAB.

clear; close all; clc;

%% Função que implementa um passo do algoritmo (sem persistent interno)
function [q_ref, dq_ref, diag_out, dq_prev_out] = ik_dls_step(...
    p_des, v_des, p_at, q_at, dq_prev, J_in, Kp_task, passo_tempo, vmax, fator_amortecimento)

    % normaliza Jacobiana para 3x3
    if ndims(J_in) == 3
        J_full = J_in(:,:,end);
    elseif isequal(size(J_in), [6 3])
        J_full = J_in(1:3,:);
    else
        J_full = J_in;
    end
    J = reshape(J_full, 3, 3);

    % entrada / erro cartesiano
    p_des = reshape(p_des(:),3,1);
    v_des = reshape(v_des(:),3,1);
    p_at  = reshape(p_at(:),3,1);
    q_at  = reshape(q_at(:),3,1);

    if isscalar(Kp_task)
        Kp_task = Kp_task * eye(3);
    end

    erro_pos = p_des - p_at;
    vel_cart = v_des + Kp_task * erro_pos;

    % Condicionamento e lambda
    condJ = cond(J);
    if ~isfinite(condJ) || condJ < 1.0
        condJ = 1.0;
    end
    c_scale = 1e-2;
    lambda = max(fator_amortecimento, c_scale * condJ);
    lambda = min(lambda, 1e2); % limite prático para testes

    JJt = J * J.';
    lambda2I = (lambda^2) * eye(3);
    matriz_intermediaria = JJt + lambda2I;

    % regularização adicional baseada em rcond
    rcond_val = rcond(matriz_intermediaria);
    reg_eps = 1e-9;
    if ~isfinite(rcond_val) || rcond_val < 1e-12
        matriz_intermediaria = matriz_intermediaria + reg_eps * eye(3);
        rcond_val = rcond(matriz_intermediaria);
        if ~isfinite(rcond_val) || rcond_val < 1e-15
            matriz_intermediaria = matriz_intermediaria + (10*reg_eps) * eye(3);
        end
    end

    % pseudo-inversa amortecida
    Jsharp = J.' / (JJt + lambda2I);
    dq_pre_null = Jsharp * vel_cart;

    % null-space posture term
    n = 3;
    q0 = zeros(n,1);
    Kp_post = diag([0.5, 0.5, 0.5]);
    N = eye(n) - Jsharp * J;

    q_err = q0 - q_at;
    q_err_wrapped = atan2(sin(q_err), cos(q_err));
    dq_ref = dq_pre_null + N * (Kp_post * q_err_wrapped);

    % informações pre-filter
    norm_pre_filter = norm(dq_ref);

    % filtro exponencial
    Tf = 0.2;
    if passo_tempo <= 0
        alpha = 0.0;
    else
        alpha = exp(-passo_tempo / Tf);
    end
    dq_ref = alpha * dq_prev + (1 - alpha) * dq_ref;
    dq_prev_out = dq_ref;

    % saturação de velocidade
    if isscalar(vmax)
        vmax_vec = repmat(abs(vmax), n, 1);
    else
        vmax_vec = abs(vmax(:));
    end
    dq_ref = max(min(dq_ref, vmax_vec), -vmax_vec);

    % integração explícita para referência posicional
    q_ref = q_at + dq_ref * passo_tempo;

    % wrapToPi
    for ii = 1:n
        q_ref(ii) = atan2(sin(q_ref(ii)), cos(q_ref(ii)));
    end

    % saturação posição
    q_min = [-2*pi; -pi; -pi];
    q_max = [ 2*pi;  pi;  pi];
    q_ref = max(min(q_ref, q_max), q_min);

    % diagnostics
    diag_out.condJ = condJ;
    diag_out.lambda = lambda;
    diag_out.rcond = rcond_val;
    diag_out.norm_pre_filter = norm_pre_filter;
    diag_out.dq_ref = dq_ref;
    diag_out.q_ref = q_ref;
end

%% Parâmetros comuns e casos de teste
passo_tempo = 1e-3;
Tsim = 0.5; % segundos
Nsteps = ceil(Tsim / passo_tempo);
tvec = (0:Nsteps-1)' * passo_tempo;

Kp_task = 5.0;
vmax = [5; 5; 5]; % rad/s
fator_amortecimento_default = 1e-6;

% inicializações para armazenar histórico
cases = {'zero_error', 'small_disp', 'singular_J', 'high_damping'};
results = struct();

for c = 1:numel(cases)
    % inicial conditions
    q_at = [0; 0; 0];
    dq_prev = zeros(3,1);
    p_at = [0; 0; 0];

    switch cases{c}
        case 'zero_error'
            p_des = p_at;
            v_des = [0;0;0];
            J = eye(3);
            damp = fator_amortecimento_default;
        case 'small_disp'
            p_des = p_at + [0.01; 0; 0]; % 1 cm no x
            v_des = [0;0;0];
            J = eye(3);
            damp = fator_amortecimento_default;
        case 'singular_J'
            p_des = p_at + [0.01; 0.01; 0];
            v_des = [0;0;0];
            J = [1 0 0; 0 0 0; 0 0 0]; % singular
            damp = fator_amortecimento_default;
        case 'high_damping'
            p_des = p_at + [0.02; 0; 0];
            v_des = [0;0;0];
            J = eye(3);
            damp = 1.0; % grande amortecimento
    end

    H_q = zeros(Nsteps, 3);
    H_dq = zeros(Nsteps, 3);
    H_lambda = zeros(Nsteps, 1);
    H_condJ = zeros(Nsteps, 1);

    for k = 1:Nsteps
        [q_ref, dq_ref, diag_out, dq_prev] = ik_dls_step(...
            p_des, v_des, p_at, q_at, dq_prev, J, Kp_task, passo_tempo, vmax, damp);

        % atualizar "plant" articular nominalmente integrando (simples)
        q_at = q_ref; % assumir que as juntas seguem a referência (teste do bloco)
        % armazenar
        H_q(k,:) = q_ref.';
        H_dq(k,:) = dq_ref.';
        H_lambda(k) = diag_out.lambda;
        H_condJ(k) = diag_out.condJ;
    end

    results.(cases{c}).t = tvec;
    results.(cases{c}).q = H_q;
    results.(cases{c}).dq = H_dq;
    results.(cases{c}).lambda = H_lambda;
    results.(cases{c}).condJ = H_condJ;
end

%% Plots de resultados (rápido)
figure('Name','IK DLS - posições articulares (rad)');
for i = 1:numel(cases)
    subplot(numel(cases),1,i);
    plot(results.(cases{i}).t, results.(cases{i}).q);
    title(cases{i}, 'Interpreter', 'none');
    ylabel('q (rad)');
    legend('q1','q2','q3');
    grid on;
end
xlabel('tempo (s)');

figure('Name','IK DLS - velocidades articulares (rad/s)');
for i = 1:numel(cases)
    subplot(numel(cases),1,i);
    plot(results.(cases{i}).t, results.(cases{i}).dq);
    title(cases{i}, 'Interpreter', 'none');
    ylabel('dq (rad/s)');
    legend('dq1','dq2','dq3');
    grid on;
end
xlabel('tempo (s)');

figure('Name','condJ e lambda');
subplot(2,1,1); hold on; grid on;
for i = 1:numel(cases); plot(results.(cases{i}).t, results.(cases{i}).condJ); end
legend(cases,'Interpreter','none'); ylabel('condJ');
subplot(2,1,2); hold on; grid on;
for i = 1:numel(cases); plot(results.(cases{i}).t, results.(cases{i}).lambda); end
ylabel('lambda'); xlabel('tempo (s)');

%% Relatório rápido de valores finais
fprintf('Resumo final (valores ao final da simulação):\n');
for i = 1:numel(cases)
    qf = results.(cases{i}).q(end,:);
    dqf = results.(cases{i}).dq(end,:);
    condf = results.(cases{i}).condJ(end);
    lambdaf = results.(cases{i}).lambda(end);
    fprintf('Case %s: q_end = [%g %g %g], dq_end = [%g %g %g], condJ = %g, lambda = %g\n', ...
        cases{i}, qf(1), qf(2), qf(3), dqf(1), dqf(2), dqf(3), condf, lambdaf);
end


