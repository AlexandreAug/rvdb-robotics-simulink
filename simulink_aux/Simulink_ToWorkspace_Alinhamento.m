%% Simulink_ToWorkspace_Alinhamento.m

clear;
clc;
close all;

mdl = 'Simulink_RVDB';

% (A) garante que o tempo "tout" será salvo pelo Simulink
load_system(mdl);
set_param(mdl, ...
    'SaveTime', 'on', ...
    'TimeSaveName', 'tout', ...
    'ReturnWorkspaceOutputs', 'on');

% (B) roda o modelo e garante retorno das variáveis no SimulationOutput
out = sim(mdl, ...
    'ReturnWorkspaceOutputs', 'on');

%% ===== TEMPO GLOBAL (prioridade: tempo_global; fallback: tout) =====
t_global = [];

% 1) tenta tempo_global dentro de out (SimulationOutput ou struct)
if tem_var(out,'tempo_global')
    sig = pegar_var(out,'tempo_global');
    [t_tmp, y_tmp] = extrair_ts(sig);
    if ~isempty(t_tmp)
        t_global = t_tmp(:);
    elseif ~isempty(y_tmp)
        t_global = y_tmp(:);
    end
end

% 2) fallback: tout (agora deve existir)
if isempty(t_global)
    if tem_var(out,'tout')
        t_global = pegar_var(out,'tout');
        t_global = t_global(:);
    else
        error('Nao encontrei tempo_global nem tout. Verifique SaveTime=on e o To Workspace do tempo_global.');
    end
end

% Mantém compatibilidade com nomes antigos
t_RVDB = t_global;

%% ===== MODO DE OPERACAO (para t_switch no pós-processamento) =====
t_modo = [];
modo_operacao = [];

if tem_var(out,'modo_operacao')
    sig = pegar_var(out,'modo_operacao');
    [t_modo, modo_operacao] = extrair_ts(sig);
end

% se veio vazio -> NaN
if isempty(modo_operacao)
    modo_operacao = NaN(size(t_global));
elseif isempty(t_modo)
    % assume alinhado por índice
    Nmin = min(numel(modo_operacao), numel(t_global));
    modo_operacao = modo_operacao(1:Nmin);
    t_modo = t_global(1:Nmin);
else
    % reamostra no tempo global preservando degrau
    modo_operacao = interp1(t_modo(:), modo_operacao(:), t_global(:), 'previous', 'extrap');
end

%% ====== DOCKING/ALINHAMENTO (Structure with time) ======
[t_alinhamento_pos_chaser_LVLH,         alinhamento_pos_chaser_LVLH]         = ts_xy(out.alinhamento_pos_chaser_LVLH);
[t_alinhamento_pos_alvo_ref_f0,         alinhamento_pos_alvo_ref_f0]         = ts_xy(out.alinhamento_pos_alvo_ref_f0);
[t_alinhamento_pos_atual_ferramenta,    alinhamento_pos_atual_ferramenta]    = ts_xy(out.alinhamento_pos_atual_ferramenta);
[t_alinhamento_fk_homogeneas,           alinhamento_fk_homogeneas]           = ts_xy(out.alinhamento_fk_homogeneas);

[t_alinhamento_pos_ref_f0_saida,        alinhamento_pos_ref_f0_saida]        = ts_xy(out.alinhamento_pos_ref_f0_saida);
[t_alinhamento_insercao,                alinhamento_insercao]                = ts_xy(out.alinhamento_insercao);
[t_alinhamento_estado_sequenciador,     alinhamento_estado_sequenciador]     = ts_xy(out.alinhamento_estado_sequenciador);
[t_alinhamento_acoplamento_ok,          alinhamento_acoplamento_ok]          = ts_xy(out.alinhamento_acoplamento_ok);
[t_alinhamento_angulo_graus,            alinhamento_angulo_graus]            = ts_xy(out.alinhamento_angulo_graus);
[t_alinhamento_prod_escalar_alinh,      alinhamento_prod_escalar_alinh]      = ts_xy(out.alinhamento_prod_escalar_alinh);
[t_alinhamento_eixo_ferramenta_f0,      alinhamento_eixo_ferramenta_f0]      = ts_xy(out.alinhamento_eixo_ferramenta_f0);
[t_alinhamento_normal_porta_f0,         alinhamento_normal_porta_f0]         = ts_xy(out.alinhamento_normal_porta_f0);
[t_alinhamento_erro_pos_ferramenta_m,   alinhamento_erro_pos_ferramenta_m]   = ts_xy(out.alinhamento_erro_pos_ferramenta_m);
[t_alinhamento_erro_pos_ferramenta_cm,  alinhamento_erro_pos_ferramenta_cm]  = ts_xy(out.alinhamento_erro_pos_ferramenta_cm);
[t_alinhamento_norma_erro_pos_ferramenta_m, alinhamento_norma_erro_pos_ferramenta_m] = ts_xy(out.alinhamento_norma_erro_pos_ferramenta_m);
[t_alinhamento_cond_pos_ok,             alinhamento_cond_pos_ok]             = ts_xy(out.alinhamento_cond_pos_ok);
[t_alinhamento_cond_angulo_ok,          alinhamento_cond_angulo_ok]          = ts_xy(out.alinhamento_cond_angulo_ok);

[t_alinhamento_instante_acoplamento,    alinhamento_instante_acoplamento]    = ts_xy(out.alinhamento_instante_acoplamento);
[t_alinhamento_insercao_real,           alinhamento_insercao_real]           = ts_xy(out.alinhamento_insercao_real);

%% ====== Distância do EEF ao centro LVLH (0,0,0) ======
[t_eef_posicao_LVLH, eef_posicao_LVLH] = ts_xy(out.eef_posicao_LVLH);
t_dist_eef_LVLH = t_eef_posicao_LVLH;
dist_eef_LVLH   = vecnorm(eef_posicao_LVLH, 2, 2);

%% ===== SALVAR =====
save('alinhamento_RVDB.mat', ...
    't_RVDB', ...                 % compatibilidade (igual ao tempo_global)
    't_global', ...               % novo (tempo global "oficial" do Clock)
    'modo_operacao', ...          % reamostrado em t_global
    't_alinhamento_pos_chaser_LVLH','alinhamento_pos_chaser_LVLH', ...
    't_alinhamento_pos_alvo_ref_f0','alinhamento_pos_alvo_ref_f0', ...
    't_alinhamento_pos_atual_ferramenta','alinhamento_pos_atual_ferramenta', ...
    't_alinhamento_fk_homogeneas','alinhamento_fk_homogeneas', ...
    't_alinhamento_pos_ref_f0_saida','alinhamento_pos_ref_f0_saida', ...
    't_alinhamento_insercao','alinhamento_insercao', ...
    't_alinhamento_estado_sequenciador','alinhamento_estado_sequenciador', ...
    't_alinhamento_acoplamento_ok','alinhamento_acoplamento_ok', ...
    't_alinhamento_angulo_graus','alinhamento_angulo_graus', ...
    't_alinhamento_prod_escalar_alinh','alinhamento_prod_escalar_alinh', ...
    't_alinhamento_eixo_ferramenta_f0','alinhamento_eixo_ferramenta_f0', ...
    't_alinhamento_normal_porta_f0','alinhamento_normal_porta_f0', ...
    't_alinhamento_erro_pos_ferramenta_m','alinhamento_erro_pos_ferramenta_m', ...
    't_alinhamento_erro_pos_ferramenta_cm','alinhamento_erro_pos_ferramenta_cm', ...
    't_alinhamento_norma_erro_pos_ferramenta_m','alinhamento_norma_erro_pos_ferramenta_m', ...
    't_alinhamento_cond_pos_ok','alinhamento_cond_pos_ok', ...
    't_alinhamento_cond_angulo_ok','alinhamento_cond_angulo_ok', ...
    't_alinhamento_instante_acoplamento','alinhamento_instante_acoplamento', ...
    't_alinhamento_insercao_real','alinhamento_insercao_real', ...
    't_dist_eef_LVLH','dist_eef_LVLH');

%% ------------------------------------------------------------------------
function [t, M] = ts_xy(sig)
    t = [];
    M = [];

    if isa(sig,'timeseries')
        t = sig.Time(:);
        X = squeeze(sig.Data);
        if isvector(X), X = X(:); end
        M = X;
        return;
    end

    if isstruct(sig) && isfield(sig,'time')
        t = sig.time(:);
        M = ts_vec(sig);
        N = min(numel(t), size(M,1));
        t = t(1:N);
        M = M(1:N,:);
        return;
    end

    error('ts_xy: sinal nao e timeseries nem struct com time.');
end

function M = ts_vec(sig)
    X = sig.signals.values;
    s = size(X);

    if numel(s) == 3 && s(2) == 1
        dim = s(1);
        N   = s(3);
        M   = reshape(permute(X, [3 1 2]), [N dim]);

    elseif numel(s) == 2
        if (s(1) == 1 || s(2) == 1)
            if s(1) == 1 && s(2) > 1
                M = X.';
            else
                M = X;
            end
        else
            if s(1) <= 10 && s(1) < s(2)
                M = X.';
            else
                M = X;
            end
        end

    else
        M = X;
    end
end

function tf = tem_var(out, nome)
    tf = false;

    if isa(out,'Simulink.SimulationOutput')
        try
            tf = any(strcmp(out.who, nome));
        catch
            % fallback simples
            try
                out.(nome); 
                tf = true;
            catch
                tf = false;
            end
        end
        return;
    end

    if isstruct(out)
        tf = isfield(out, nome);
        return;
    end
end

function v = pegar_var(out, nome)
    if isa(out,'Simulink.SimulationOutput')
        % para variáveis de To Workspace e também para tout
        try
            v = out.get(nome);
        catch
            v = out.(nome);
        end
        return;
    end

    if isstruct(out)
        v = out.(nome);
        return;
    end

    error('pegar_var: tipo de "out" nao suportado.');
end

function [t_out, y_out] = extrair_ts(sig)
    % Extrai tempo e valores de: timeseries, struct (To Workspace), vetor numérico.
    t_out = [];
    y_out = [];

    if isempty(sig)
        return;
    end

    if isa(sig,'timeseries')
        t_out = sig.Time(:);
        y_out = squeeze(sig.Data);
        if isvector(y_out), y_out = y_out(:); end
        return;
    end

    if isstruct(sig)
        if isfield(sig,'time'), t_out = sig.time(:); end

        if isfield(sig,'signals')
            if isfield(sig.signals,'values')
                y_out = sig.signals.values;
            else
                y_out = sig.signals(1).values;
            end
            y_out = squeeze(y_out);
            if isvector(y_out), y_out = y_out(:); end
            return;
        end

        if isfield(sig,'Data')
            y_out = squeeze(sig.Data);
            if isvector(y_out), y_out = y_out(:); end
            return;
        end

        return;
    end

    if isnumeric(sig)
        y_out = sig;
        if isvector(y_out), y_out = y_out(:); end
        return;
    end
end
