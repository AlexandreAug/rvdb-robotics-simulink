function [q_ref, dq_ref, diag_out, dq_prev_out] = ik_dls_step(...
    p_des, v_des, p_at, q_at, dq_prev, J_in, Kp_task, passo_tempo, vmax, fator_amortecimento)
% IK_DLS_STEP  — um passo da cinemática inversa diferencial (DLS + null-space + filtro + integração)
% Entradas (vetores 3x1 ou escalares conforme nome) — saída: q_ref (posição referência), dq_ref (vel ref), diag_out (struct), dq_prev_out

    % Normaliza Jacobiana
    if ndims(J_in) == 3
        J_full = J_in(:,:,end);
    elseif isequal(size(J_in), [6 3])
        J_full = J_in(1:3,:);
    else
        J_full = J_in;
    end
    J = reshape(J_full, 3, 3);

    % parâmetros internos (ajustáveis)
    if isscalar(Kp_task)
        Kp_task = Kp_task * eye(3);
    end
    c_scale = 1;      % alterar para 1.0 para testar efeito de condicionamento
    Kp_post = diag([0.5,0.5,0.5]);
    Tf = 0.2;
    reg_eps = 1e-9;
    n = 3;

    % erro cartesiano
    p_des = reshape(p_des(:),3,1);
    v_des = reshape(v_des(:),3,1);
    p_at  = reshape(p_at(:),3,1);
    q_at  = reshape(q_at(:),3,1);

    vel_cart = v_des + Kp_task * (p_des - p_at);

    % condicionamento e lambda
    condJ = cond(J);
    if ~isfinite(condJ) || condJ < 1.0
        condJ = 1.0;
    end
    lambda = max(fator_amortecimento, c_scale * condJ);

    JJt = J * J.';
    lambda2I = (lambda^2) * eye(3);
    matriz_intermediaria = JJt + lambda2I;

    % regularização baseada em rcond
    rcond_val = rcond(matriz_intermediaria);
    if ~isfinite(rcond_val) || rcond_val < 1e-12
        matriz_intermediaria = matriz_intermediaria + reg_eps * eye(3);
        rcond_val = rcond(matriz_intermediaria);
        if ~isfinite(rcond_val) || rcond_val < 1e-15
            matriz_intermediaria = matriz_intermediaria + (10*reg_eps) * eye(3);
        end
    end

    % pseudo-inversa amortecida
    Jsharp = J.' / (JJt + lambda2I);
    dq_pre = Jsharp * vel_cart;

    % null-space posture
    q0 = zeros(n,1);
    N = eye(n) - Jsharp * J;
    q_err = q0 - q_at;
    q_err_wrapped = atan2(sin(q_err), cos(q_err));
    dq_ref = dq_pre + N * (Kp_post * q_err_wrapped);

    % filtro exponencial
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

    % integração (um passo)
    q_ref = q_at + dq_ref * passo_tempo;

    % wrapToPi e saturação de posição
    for ii = 1:n
        q_ref(ii) = atan2(sin(q_ref(ii)), cos(q_ref(ii)));
    end
    q_min = [-2*pi; -pi; -pi];
    q_max = [ 2*pi;  pi;  pi];
    q_ref = max(min(q_ref, q_max), q_min);

    % diagnostics
    diag_out.condJ = condJ;
    diag_out.lambda = lambda;
    diag_out.rcond = rcond_val;
    diag_out.norm_dq_pre = norm(dq_pre);
    diag_out.norm_dq_ref = norm(dq_ref);
    diag_out.Jsharp_fro = norm(Jsharp,'fro');
end
