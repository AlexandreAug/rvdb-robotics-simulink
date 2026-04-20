%% Graficos_4_Alinhamento.m
clc;
close all;

load('alinhamento_RVDB.mat');

% TEMPO PADRÃO (Clock)
t     = t_global(:);
t_fim = t(end);

% Troca de modo (0->1) em t
idx_switch = find(modo_operacao(:) >= 0.5, 1, 'first');
t_switch   = t(idx_switch);

% Instante de acoplamento (valor)
t_acoplamento = alinhamento_instante_acoplamento(end);

%% ESTILO
fs = 20;
set(groot,'defaultAxesFontSize',   fs);
set(groot,'defaultTextFontSize',   fs);
set(groot,'defaultLegendFontSize', fs);
set(0, 'DefaultLineLineWidth', 2);

lw_vline = 3;
ls_mr    = 'm';   % início MR
ls_ac    = 'g';   % acoplamento

%% ====== REAMOSTRAR TUDO EM t ======
ang   = reamostrar_linear(t, t_alinhamento_angulo_graus,            alinhamento_angulo_graus);
dotp  = reamostrar_linear(t, t_alinhamento_prod_escalar_alinh,      alinhamento_prod_escalar_alinh);
s_cmd = reamostrar_linear(t, t_alinhamento_insercao,                alinhamento_insercao);
s_real= reamostrar_linear(t, t_alinhamento_insercao_real,           alinhamento_insercao_real);
ep    = reamostrar_linear(t, t_alinhamento_norma_erro_pos_ferramenta_m, alinhamento_norma_erro_pos_ferramenta_m);

e_pos_m  = reamostrar_linear(t, t_alinhamento_erro_pos_ferramenta_m,  alinhamento_erro_pos_ferramenta_m);
e_pos_cm = reamostrar_linear(t, t_alinhamento_erro_pos_ferramenta_cm, alinhamento_erro_pos_ferramenta_cm);

pref0 = reamostrar_linear(t, t_alinhamento_pos_ref_f0_saida,        alinhamento_pos_ref_f0_saida);
pat0  = reamostrar_linear(t, t_alinhamento_pos_atual_ferramenta,    alinhamento_pos_atual_ferramenta);

e_tool = reamostrar_linear(t, t_alinhamento_eixo_ferramenta_f0, alinhamento_eixo_ferramenta_f0);
n_porta = reamostrar_linear(t, t_alinhamento_normal_porta_f0, alinhamento_normal_porta_f0);

% normalizar para evitar "medição" enganosa se algum vetor não vier unitário
e_tool_u = e_tool ./ max(vecnorm(e_tool,2,2), 1e-12);
n_porta_u = n_porta ./ max(vecnorm(n_porta,2,2), 1e-12);

% contribuições por eixo para o produto escalar
c_xyz = e_tool_u .* n_porta_u;   % [Nx3]


%% 1) Ângulo de alinhamento
% Mede o erro angular entre o eixo da ferramenta e a normal da porta

figure; hold on;

h1   = plot(t, ang(:,1), 'k');

h_mr = xline(t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac = xline(t_acoplamento, ls_ac, 'LineWidth', lw_vline);
% h_lim = yline(10, 'r--', 'LineWidth', 2);   % Limite em 10°

grid on;
xlabel('Tempo [s]');
ylabel('Ângulo de alinhamento da ferramenta no ref. 0 [grau]');

xlim([t(1) t_fim]);
legend([h1, h_mr, h_ac], ...
       {'Ângulo de alinhamento da ferramenta', ...
        'Início do modo operacional do MR', ...
        'Instante de acoplamento'}, ...
       'Location','eastoutside');
hold off;

%% 2) Componentes do eixo da ferramenta e da normal da porta em {0}
% 2a: eixo da ferramenta (x0,y0,z0)
% 2b: normal da porta (x0,y0,z0)

figure;

% tlo = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% ---------------- 2a) Eixo da ferramenta ----------------
ax1 = nexttile;
hold(ax1,'on');

h_ex = plot(ax1, t, e_tool_u(:,1), 'k');
h_ey = plot(ax1, t, e_tool_u(:,2), 'r');
h_ez = plot(ax1, t, e_tool_u(:,3), 'b');

h_mr1 = xline(ax1, t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac1 = xline(ax1, t_acoplamento, ls_ac, 'LineWidth', lw_vline);

grid(ax1,'on');
ylabel(ax1,'Eixo da ferramenta no ref. 0');
xlim(ax1,[t(1) t_fim]);
ylim(ax1,[-1.1 1.1]);

legend(ax1, [h_ex h_ey h_ez h_mr1 h_ac1], ...
       {'Eixo da ferramenta (x, ref. 0)',...
       'Eixo da ferramenta (y, ref. 0)',...
       'Eixo da ferramenta (z, ref. 0)', ...
        'Início do modo operacional do MR','Instante de acoplamento'}, ...
       'Location','eastoutside');

% ---------------- 2b) Normal da porta ----------------
% ax2 = nexttile; hold(ax2,'on');
% 
% h_nx = plot(ax2, t, n_porta_u(:,1), 'k--');
% h_ny = plot(ax2, t, n_porta_u(:,2), 'r--');
% h_nz = plot(ax2, t, n_porta_u(:,3), 'b--');
% 
% h_mr2 = xline(ax2, t_switch,      ls_mr, 'LineWidth', lw_vline);
% h_ac2 = xline(ax2, t_acoplamento, ls_ac, 'LineWidth', lw_vline);
% 
% grid(ax2,'on');
% xlabel(ax2,'Tempo [s]');
% ylabel(ax2,'Normal da porta no ref. 0');
% xlim(ax2,[t(1) t_fim]);
% ylim(ax2,[-0.6 1.1]);
% 
% legend(ax2, [h_nx h_ny h_nz h_mr2 h_ac2], ...
%        {'Normal da porta (x)',...
%        'Normal da porta (y)',...
%        'Normal da porta (z)', ...
%         'Início do modo operacional do MR','Instante de acoplamento'}, ...
%        'Location','eastoutside');
% 
% % mesma escala de tempo
% linkaxes([ax1 ax2],'x');


%% 3) Produto escalar (e_tool · n_porta)

figure; hold on;
h2   = plot(t, dotp(:,1), 'k');
h_mr = xline(t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac = xline(t_acoplamento, ls_ac, 'LineWidth', lw_vline);
grid on;

xlabel('Tempo [s]'); ylabel('Produto escalar do alinhamento da ferramenta no ref. 0');
xlim([t(1) t_fim]); ylim([-0.5 1.1]);              

legend([h2, h_mr, h_ac], ...
       {'Produto escalar do alinhamento', 'Início do modo operacional do MR', 'Instante de acoplamento'}, ...
       'Location','eastoutside');
hold off;

%% 4) Contribuições por eixo no produto escalar: (e_i * n_i)
figure; hold on;

hcx = plot(t, c_xyz(:,1), 'k');
hcy = plot(t, c_xyz(:,2), 'r');
hcz = plot(t, c_xyz(:,3), 'b');
hs  = plot(t, sum(c_xyz,2), 'g--');  % deve coincidir com dotp (se vetores unitários)

h_mr = xline(t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac = xline(t_acoplamento, ls_ac, 'LineWidth', lw_vline);

grid on;
xlabel('Tempo [s]');
ylabel('Contribuição no produto escalar (por eixo) no ref. 0');

xlim([t(1) t_fim]);
ylim([-0.5 1.1]);

legend([hcx hcy hcz hs h_mr h_ac], { ...
 'Contribuição (x, ref. 0)', ...
 'Contribuição (y, ref. 0)', ...
 'Contribuição (z, ref. 0)', ...
 'Soma das contribuições (produto escalar total)', ...
 'Início do modo operacional do MR', ...
 'Instante de acoplamento'}, ...
 'Location','eastoutside');

hold off;


% %% 5) Inserção: comando e real
% % Compara o que o sequenciador está pedindo (perfil de aproximação/inserção)
% % com o que a cinemática/dinâmica está efetivamente realizando
% 
% figure; hold on;
% h3a  = plot(t, s_cmd(:,1),  'k');
% h3b  = plot(t, s_real(:,1), 'r');
% h_mr = xline(t_switch, ls_mr, 'LineWidth', lw_vline);
% grid on; xlabel('Tempo [s]'); ylabel('Distância restante ao alvo');
% xlim([t(1) t_fim]);
% legend([h3a, h3b, h_mr], ...
%        {'Comando (sequenciador) – distância restante', ...
%         'Medida real – distância restante', ...
%         'Início do modo operacional do MR'}, ...
%        'Location','eastoutside');
% hold off;

%% 5) Erro de posição da ferramenta: magnitude e componentes (uma única figure)

fig5 = figure;
set(fig5,'DefaultAxesFontSize',   fs);
set(fig5,'DefaultTextFontSize',   fs);
set(fig5,'DefaultLegendFontSize', fs);

tlo = tiledlayout(fig5, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% reamostra em metros (garante consistência)
e_pos_m = reamostrar_linear(t, t_alinhamento_erro_pos_ferramenta_m, alinhamento_erro_pos_ferramenta_m);

% ===================== (5-1) MAGNITUDE =====================
ax1 = nexttile(tlo); hold(ax1,'on');

h_mag = plot(ax1, t, ep(:,1), 'k');
h_mr1 = xline(ax1, t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac1 = xline(ax1, t_acoplamento, ls_ac, 'LineWidth', lw_vline);

grid(ax1,'on');
xlabel(ax1,'Tempo [s]');
xlim(ax1,[t(1) t_fim]);

leg1 = legend(ax1, [h_mag h_mr1 h_ac1], ...
    {'Magnitude do erro da posição do efetuador final',...
     'Início da operação do MR','Instante do acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');
set(leg1,'FontSize',fs);

axes(ax1);
aplicar_ylim_sem_picos(ep(:,1), 1, 99, 0.10);

hold(ax1,'off');

% ===================== (5-2) COMPONENTES (COMPLETO) =====================
ax2 = nexttile(tlo); hold(ax2,'on');

h_ex = plot(ax2, t, e_pos_m(:,1), 'k');
h_ey = plot(ax2, t, e_pos_m(:,2), 'r');
h_ez = plot(ax2, t, e_pos_m(:,3), 'b');

h_mr2 = xline(ax2, t_switch,      ls_mr, 'LineWidth', lw_vline);
h_ac2 = xline(ax2, t_acoplamento, ls_ac, 'LineWidth', lw_vline);

grid(ax2,'on');
xlabel(ax2,'Tempo [s]');
xlim(ax2,[t(1) t_fim]);
ylabel(ax2,'Erro de posição do efetuador final [m]');

leg2 = legend(ax2, [h_ex h_ey h_ez h_mr2 h_ac2], ...
    {'Erro de posição do efetuador final (x, ref. 0)',...
     'Erro de posição do efetuador final (y, ref. 0)',...
     'Erro de posição do efetuador final (z, ref. 0)',...
     'Início da operação do MR',...
     'Instante do acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');
set(leg2,'FontSize',fs);

hold(ax2,'off');

% ===================== (5-3) COMPONENTES (ZOOM 0 a 10 s) =====================
ax3 = nexttile(tlo); hold(ax3,'on');

plot(ax3, t, e_pos_m(:,1), 'k');
plot(ax3, t, e_pos_m(:,2), 'r');
plot(ax3, t, e_pos_m(:,3), 'b');

xline(ax3, t_switch,      ls_mr, 'LineWidth', lw_vline);
xline(ax3, t_acoplamento, ls_ac, 'LineWidth', lw_vline);

grid(ax3,'on');
xlabel(ax3,'Tempo [s]');
% ylabel(ax3,'Erro de posição da ferramenta por eixo [m]');
xlim(ax3,[0 100]);

hold(ax3,'off');

% garante fonte 16 nos eixos e em todos os textos da figure
set([ax1 ax2 ax3],'FontSize',fs);
set(findall(fig5,'Type','Text'),'FontSize',fs);


%% 6) Posição em {0}: referência vs atual (x,y,z)
% 8a: série completa | 8b: zoom (400 a 700 s) com escala Y fixa [-0.5, 1]
% - ylabel único centralizado
% - legenda topo: somente linhas CHEIAS + MR + acoplamento
% - legenda baixo: somente linhas TRACEJADAS + MR + acoplamento
% - usar x y z
% - fonte 16 garantida

fig6 = figure;
set(fig6,'DefaultAxesFontSize',fs,'DefaultTextFontSize',fs,'DefaultLegendFontSize',fs);

tlo = tiledlayout(fig6,2,1,'TileSpacing','compact','Padding','compact');
% ylabel(tlo,'Posição do efetuador final no frame {0} da base [m]');

% ===================== 6a) SÉRIE COMPLETA =====================
ax1 = nexttile(tlo); hold(ax1,'on');

h_refx = plot(ax1,t,pref0(:,1),'k');
h_refy = plot(ax1,t,pref0(:,2),'r');
h_refz = plot(ax1,t,pref0(:,3),'b');

% atuais (visual apenas; não entram na legenda do topo)
plot(ax1,t,pat0(:,1),'k--','HandleVisibility','off');
plot(ax1,t,pat0(:,2),'r--','HandleVisibility','off');
plot(ax1,t,pat0(:,3),'b--','HandleVisibility','off');

h_mr1 = xline(ax1,t_switch,ls_mr,'LineWidth',lw_vline);
h_ac1 = xline(ax1,t_acoplamento,ls_ac,'LineWidth',lw_vline);

grid(ax1,'on');
xlabel(ax1,'Tempo [s]');
xlim(ax1,[t(1) t_fim]);

leg6a = legend(ax1,[h_refx h_refy h_refz h_mr1 h_ac1], ...
    {'Posição desejada do efetuador final (x, ref. 0)', ...
     'Posição desejada do efetuador final (y, ref. 0)', ...
     'Posição desejada do efetuador final (z, ref. 0)', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');
set(leg6a,'FontSize',fs);

hold(ax1,'off');

% ===================== 6b) ZOOM (400 a 700 s) =====================
ax2 = nexttile(tlo); hold(ax2,'on');

% referência (CONTÍNUO) — aparece no gráfico, mas NÃO entra na legenda de baixo
plot(ax2,t,pref0(:,1),'k','HandleVisibility','off');
plot(ax2,t,pref0(:,2),'r','HandleVisibility','off');
plot(ax2,t,pref0(:,3),'b','HandleVisibility','off');

% atual (TRACEJADO) — entra na legenda de baixo
h_atux = plot(ax2,t,pat0(:,1),'k--');
h_atuy = plot(ax2,t,pat0(:,2),'r--');
h_atuz = plot(ax2,t,pat0(:,3),'b--');

h_mr2 = xline(ax2,t_switch,ls_mr,'LineWidth',lw_vline);
h_ac2 = xline(ax2,t_acoplamento,ls_ac,'LineWidth',lw_vline);

grid(ax2,'on');
xlabel(ax2,'Tempo [s]');
xlim(ax2,[400 700]);
ylim(ax2,[-0.5 1.5]);
ylabel('Posição do efetuador final no ref. 0 [m]');

leg6b = legend(ax2,[h_atux h_atuy h_atuz h_mr2 h_ac2], ...
    {'Posição real do efetuador final (x, ref. 0)',...
    'Posição real do efetuador final (y, ref. 0)',...
    'Posição real do efetuador final (z, ref. 0)', ...
     'Início do modo operacional do MR','Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');
set(leg6b,'FontSize',fs);

hold(ax2,'off');

set([ax1 ax2],'FontSize',fs);
set(findall(fig6,'Type','Text'),'FontSize',fs);


%% ===================== DEBUG (GRAFICOS_4_ALINHAMENTO) =====================
fprintf('\n==================== DEBUG (GRAFICOS_4_ALINHAMENTO) ====================\n');

%% 1) Resumo curto (tempo + eventos)
fprintf('Tempo (t_global): N=%d | t_ini=%.3f s | t_fim=%.3f s\n', numel(t), t(1), t(end));

if isempty(idx_switch) || ~isfinite(t_switch)
    fprintf('Evento: t_switch (modo_operacao 0->1) = NA\n');
else
    fprintf('Evento: t_switch (modo_operacao 0->1) = %.3f s (idx=%d)\n', t_switch, idx_switch);
end

fprintf('Evento: t_acoplamento = %.3f s\n', t_acoplamento);

fprintf('t_alinhamento_angulo_graus:       [%.3f, %.3f] s\n', t_alinhamento_angulo_graus(1),       t_alinhamento_angulo_graus(end));
fprintf('t_alinhamento_prod_escalar_alinh: [%.3f, %.3f] s\n', t_alinhamento_prod_escalar_alinh(1), t_alinhamento_prod_escalar_alinh(end));

%% 2) Dimensões (uma linha, só o que importa)
fprintf('\nDimensões (reamostrado em t_global):\n');
fprintf('  ang=%dx%d | dotp=%dx%d | s_cmd=%dx%d | s_real=%dx%d | ep=%dx%d\n', ...
    size(ang,1),size(ang,2), size(dotp,1),size(dotp,2), size(s_cmd,1),size(s_cmd,2), size(s_real,1),size(s_real,2), size(ep,1),size(ep,2));
fprintf('  e_pos_m=%dx%d | e_pos_cm=%dx%d | pref0=%dx%d | pat0=%dx%d\n', ...
    size(e_pos_m,1),size(e_pos_m,2), size(e_pos_cm,1),size(e_pos_cm,2), size(pref0,1),size(pref0,2), size(pat0,1),size(pat0,2));

%% 3) Amostras "mínimas" (t_ini, t_switch, t_acoplamento, t_fim) em uma linha por evento
fprintf('\nAmostras (uma linha por instante):\n');

% helper: índice mais próximo
idx_near = @(tt) find(abs(t-tt)==min(abs(t-tt)), 1, 'first');

% 3.1) t_ini
i0 = 1;
fprintf('  t_ini   = %8.3f | ang=%7.3f deg | dot=%+7.4f | ep=%8.4f m | e_cm=[%7.2f %7.2f %7.2f]\n', ...
    t(i0), ang(i0,1), dotp(i0,1), ep(i0,1), e_pos_cm(i0,1), e_pos_cm(i0,2), e_pos_cm(i0,3));

% 3.2) t_switch (se existir)
if ~isempty(idx_switch) && isfinite(t_switch)
    isw = idx_switch;
    fprintf('  t_switch= %8.3f | ang=%7.3f deg | dot=%+7.4f | ep=%8.4f m | e_cm=[%7.2f %7.2f %7.2f]\n', ...
        t(isw), ang(isw,1), dotp(isw,1), ep(isw,1), e_pos_cm(isw,1), e_pos_cm(isw,2), e_pos_cm(isw,3));
end

% 3.3) t_acoplamento (índice mais próximo)
iac = idx_near(t_acoplamento);
fprintf('  t_acopl = %8.3f | ang=%7.3f deg | dot=%+7.4f | ep=%8.4f m | e_cm=[%7.2f %7.2f %7.2f]\n', ...
    t(iac), ang(iac,1), dotp(iac,1), ep(iac,1), e_pos_cm(iac,1), e_pos_cm(iac,2), e_pos_cm(iac,3));

% 3.4) t_fim
ifin = numel(t);
fprintf('  t_fim   = %8.3f | ang=%7.3f deg | dot=%+7.4f | ep=%8.4f m | e_cm=[%7.2f %7.2f %7.2f]\n', ...
    t(ifin), ang(ifin,1), dotp(ifin,1), ep(ifin,1), e_pos_cm(ifin,1), e_pos_cm(ifin,2), e_pos_cm(ifin,3));

%% 4) Extremos (curto)
fprintf('\nExtremos (t_global):\n');
[ang_min, i_ang_min] = min(ang(:,1), [], 'omitnan');
[ang_max, i_ang_max] = max(ang(:,1), [], 'omitnan');
fprintf('  ang:  min=%7.3f deg @ %8.3f s | max=%7.3f deg @ %8.3f s\n', ang_min, t(i_ang_min), ang_max, t(i_ang_max));

[dot_min, i_dot_min] = min(dotp(:,1), [], 'omitnan');
[dot_max, i_dot_max] = max(dotp(:,1), [], 'omitnan');
fprintf('  dot:  min=%+7.4f     @ %8.3f s | max=%+7.4f     @ %8.3f s\n', dot_min, t(i_dot_min), dot_max, t(i_dot_max));

[ep_min, i_ep_min] = min(ep(:,1), [], 'omitnan');
[ep_max, i_ep_max] = max(ep(:,1), [], 'omitnan');
fprintf('  ep:   min=%8.4f m  @ %8.3f s | max=%8.4f m  @ %8.3f s\n', ep_min, t(i_ep_min), ep_max, t(i_ep_max));

%% 5) Consistência ep vs ||pat0 - pref0|| (curto)
e3d    = vecnorm(pat0 - pref0, 2, 2);
dif_ep = ep(:,1) - e3d;

[err_max, i_err_max] = max(abs(dif_ep), [], 'omitnan');
fprintf('\nConsistência: max(|ep - ||pat0-pref0|| |) = %.3g @ t=%.3f s\n', err_max, t(i_err_max));

fprintf('=======================================================================\n');

%% ================= FUNÇÕES LOCAIS =================
function Yt = reamostrar_linear(t, ts, Y)
    t  = t(:);
    ts = ts(:);
    if isvector(Y), Y = Y(:); end
    Yt = interp1(ts, Y, t, 'linear', 'extrap');
end

function aplicar_ylim_sem_picos(Y, p_lo, p_hi, margem_frac)
    y = Y(:);
    y = y(isfinite(y));
    lo = prctile(y, p_lo);
    hi = prctile(y, p_hi);
    m  = margem_frac * (hi - lo);
    ylim([lo - m, hi + m]);
end
