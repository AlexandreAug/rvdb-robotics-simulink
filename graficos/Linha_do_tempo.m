%% Linha_do_tempo.m
clear; clc;

% todos os tempos estão em segundos

%% 1) Carrega dados de outros arquivos
TransfOrbital = TransferenciaOrbital();
Graficos_3_RVDB;

% carrega a trajetória da simulação RVDB (base flutuante + MR)
load('trajetoria_RVDB.mat', 't_RVDB', 'modo_operacao');

% vetor de tempo da simulação RVDB
t_rvdb = t_RVDB(:);
modo_op = modo_operacao(:);

% instante de troca de modo (0 -> 1) na simulação de 900 s
idx_switch = find(modo_op >= 0.5, 1, 'first');
if isempty(idx_switch)
    t_switch_local = NaN;        % nunca comutou
else
    t_switch_local = t_rvdb(idx_switch);   % tempo relativo à simulação RVDB
end

% duração da aproximação final (após a troca de dinâmica)
duracao_app_final_local = t_fim_missao - t_switch_local;

%% 2a) Estabelecendo tempos (Cenário A)

% Início absoluto da missão RVD/B
% Início da aproximação longa
tempo_inicial_missao = 0; 

t_queima1         = TransfOrbital.tempo_espera; % primeiro impulso da TH
t_queima2         = TransfOrbital.tempo_burn2;  % segundo impulso da TH
tempo_drift       = TransfOrbital.dt_drift;     % duração do drift orbital
t_final_app_longa = t_queima2 + tempo_drift;    % fim da aproximação longa

% Início da aproximação curta (Hill/CW) – início da simulação RVDB (900 s)
t_inicio_app_curta = t_final_app_longa;
t_final_app_curta  = t_final_app_longa + 67.57;

% Início da aproximação final (e fim da curta)
t_inicio_app_final = t_final_app_curta;
t_final_app_final  = t_switch + t_inicio_app_final;

t_final_missao = t_final_app_final;

%% 2b) Cenário B

t_inicio_B = tempo_inicial_missao;
t_queima1_B = 31134;
t_queima2_B = 33972;
tempo_drift_B = tempo_drift;
t_final_app_longa_B = t_queima2_B + tempo_drift_B; % fim da aprox. longa

% Início da aproximação curta "B" (Hill/CW) – início da simulação RVDB
t_inicio_app_curta_B = t_final_app_longa_B;
t_final_app_curta_B  = t_final_app_longa_B + 67.57;

% Início da aproximação final (e fim da curta)
t_inicio_app_final_B = t_final_app_curta_B;
t_final_app_final_B  = t_switch + t_inicio_app_final_B;

t_final_missao_B = t_final_app_final_B;

%% 3) Instante ABSOLUTO em que o modo de operação troca de 0 -> 1
if isnan(t_switch_local)
    t_modo_MR = NaN;
else
    % t_switch_local é medido a partir do início da aproximação curta
    t_modo_MR = t_inicio_app_curta + t_switch_local;
end

%% 4) Plot da linha do tempo geral (com linhas verticais)
fs = 16;

figure;
hold on; grid on;

% linha horizontal de referência (missão inteira)
plot([tempo_inicial_missao, t_final_missao], [0 0], ...
    'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);

% ---- eventos (tempos absolutos) ----
% - início da missão                  = tempo_inicial_missao
% - início aprox. longa dist.         = t_queima1
% - início aprox. curta dist.         = t_inicio_app_curta
% - início aprox. final               = t_inicio_app_final
% - início modo operacional do MR     = t_modo_MR
% - fim aprox. final / missão         = t_final_app_final

t_events = [ ...
    tempo_inicial_missao, ...   % 1
    t_queima1, ...              % 2
    t_inicio_app_curta, ...     % 3
    t_inicio_app_final, ...     % 4
    t_modo_MR, ...              % 5
    t_final_app_final ...       % 6
];

labels = { ...
    'Início da missão de RVD/B', ...
    'Início da aprox. longa distância', ...
    'Início da aprox. curta distância', ...
    'Início da aprox. final', ...
    'Início do modo operacional do MR', ...
    'Fim da aproximação final e da missão' ...
};

nEv = numel(t_events);

% cores (uma por evento)
cores = [ ...
    0   0   0   ;  % missão início (preto)
    1   0   0   ;  % início longa (vermelho)
    0   0   1   ;  % início curta (azul)
    0   0.5 0   ;  % início final (verde)
    1   0   1   ;  % início modo MR (magenta)
    0.3 0.3 0.3 ;  % fim missão (cinza escuro)
];

% estilos de linha
estilos = {'-', '--', ':', '-.', '--', '-'};

if size(cores,1) < nEv
    cores = repmat(cores, ceil(nEv/size(cores,1)), 1);
end
if numel(estilos) < nEv
    estilos = repmat(estilos, 1, ceil(nEv/numel(estilos)));
end

ylim([-1 1]);
yl = ylim;

h_ev = gobjects(nEv,1);
for k = 1:nEv
    if k < nEv
        % linhas verticais normais
        h_ev(k) = xline(t_events(k), ...
            'Color',     cores(k,:), ...
            'LineStyle', estilos{k}, ...
            'LineWidth', 2);
    else
        % última linha (fim da aprox. final e missão) com "x" ao longo da linha
        y_line = linspace(-1, 1, 30);
        x_line = t_events(k) * ones(size(y_line));
        h_ev(k) = plot(x_line, y_line, ...
            'x-', ...
            'Color', cores(k,:), ...
            'LineWidth', 2, ...
            'MarkerSize', 6);
    end
end

xlabel('Tempo absoluto [s]');
yticks([]);

% limites e marcações do eixo de tempo
xlim([tempo_inicial_missao, 135*60]); % 0 a 135 min  
xticks(tempo_inicial_missao:500:t_final_missao);

h_leg = legend(h_ev, labels, 'Location', 'eastout');
set(h_leg, 'FontSize', fs);

set(gca, 'FontSize', fs);
set(get(gca,'XLabel'), 'FontSize', fs);
set(get(gca,'YLabel'), 'FontSize', fs);
set(get(gca,'Title'),  'FontSize', fs);



%% 5a) Impressão dos tempos no Command Window
fprintf('\n===== Linha do tempo geral "A" (valores absolutos) =====\n');

print_tempo('Início da missão:', tempo_inicial_missao);
print_tempo('Início da aprox. longa (Delta-V1):', t_queima1);
print_tempo('Final da aprox. longa (Delta-V2):',            t_queima2);
% print_tempo('Duração do drift orbital',      tempo_drift);
% print_tempo('Final do drift orbital:',             t_final_app_longa);

print_tempo('Início da aprox. curta',          t_inicio_app_curta);
% print_tempo('Fim da app. curta',             t_final_app_curta);

% instante local da comutação (dentro dos 900 s)
if exist('t_switch_local','var') && ~isnan(t_switch_local)
    % print_tempo('Comutação modo MR (tempo local)', t_switch_local);
end

print_tempo('Início da app. final',          t_inicio_app_final);

% instante absoluto da comutação (linha do tempo geral)
if exist('t_modo_MR','var') && ~isnan(t_modo_MR)
    print_tempo('Comutação entre as dinâmicas', t_modo_MR);
end

print_tempo('Fim da aprox. final',             t_final_app_final);

fprintf('\n===== Tempo final da missão =====\n');
print_tempo('Tempo total da missão',         t_final_missao);

%% 5b) Impressão dos tempos no Command Window – Cenário B
fprintf('\n===== Linha do tempo geral (valores absolutos) – Cenário B =====\n');

print_tempo('Início da missão (B):',                  t_inicio_B);
print_tempo('Início da aprox. longa (Delta-V1, B):',  t_queima1_B);
print_tempo('Final da aprox. longa (Delta-V2, B):',   t_queima2_B);

% Drift orbital (B)
print_tempo('Duração do drift orbital (B):',          tempo_drift_B);
print_tempo('Final do drift orbital (B):',            t_final_app_longa_B);

% Aproximação curta (B)
print_tempo('Início da aprox. curta (B):',            t_inicio_app_curta_B);
print_tempo('Fim da aprox. curta (B):',               t_final_app_curta_B);

% Aproximação final (B)
print_tempo('Início da aprox. final (B):',            t_inicio_app_final_B);

t_modo_MR_B = t_final_app_final_B - (t_final_app_final - t_modo_MR);

% instante absoluto da comutação (linha do tempo geral)
if exist('t_modo_MR','var') && ~isnan(t_modo_MR)
    print_tempo('Comutação entre as dinâmicas (B):', t_modo_MR_B);
end

print_tempo('Fim da aprox. final (B):',               t_final_app_final_B);

fprintf('\n===== Tempo final da missão – Cenário B =====\n');
print_tempo('Tempo total da missão (B):',             t_final_missao_B);

%% ========================================================================
%% Funções locais auxiliares
function print_tempo(rotulo, t)
    if isnan(t)
        fprintf('%-35s : NaN\n', rotulo);
    else
        fprintf('%-35s : %.0f [s] (%s)\n', rotulo, t, fmt_hms(t));
    end
end

function s = fmt_hms(t)
    h    = floor(t/3600);
    m    = floor(mod(t,3600)/60);
    ssec = floor(mod(t,60));
    s = sprintf('%02dh %02dmin %02ds', h, m, ssec);
end
