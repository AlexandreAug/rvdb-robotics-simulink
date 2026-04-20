%% Graficos_3_RVDB.m
% Geração de gráficos da simulação RVDB (base flutuante + MR)

clc;
clear;
close all;

load('trajetoria_RVDB.mat');

t_fim_plot    = tempo_final;   % [s]

%% Definições dos gráficos
fs = 20;  % fontsize para eixos, rótulos e legendas

% Fonte padrão
set(groot,'defaultAxesFontSize',   fs);
set(groot,'defaultTextFontSize',   fs);
set(groot,'defaultLegendFontSize', fs);

% Estilo básico de gráficos
set(0, 'DefaultLineLineWidth', 1.2);

% vetor de tempo padrão usado em todos os plots
t = t_RVDB(:);

% instante de troca de modo (0 -> 1), com base no degrau salvo
modo_operacao = modo_operacao(:);
idx_switch = find(modo_operacao >= 0.5, 1, 'first');

if isempty(idx_switch)
    t_switch = NaN;
else
    t_switch = t(idx_switch);
end

% ======= Instante de acoplamento =======
t_acoplamento = NaN;
try
    S_al = load('alinhamento_RVDB.mat', 'alinhamento_instante_acoplamento');
    if isfield(S_al,'alinhamento_instante_acoplamento') && ~isempty(S_al.alinhamento_instante_acoplamento)
        t_acoplamento = S_al.alinhamento_instante_acoplamento(end);
    end
catch
    t_acoplamento = NaN;
end

% fim de missão (inicializa sempre; será calculado no Bloco 9 quando plotado)
t_fim_missao = NaN;

% padrão único para a linha "Início do modo operacional do MR"
lw_mr = 2.0;            % mesma grossura em todos
mr_style = 'm';        % pontilhada preta

% padrão único para a linha "Instante de acoplamento"
lw_ac = 2.0;
ac_style = 'g';       % verde traço-ponto

%% Seletor de gráficos
plot_sel = 0;   % 0 = todos
print_sel = plot_sel; % 1 a 11 ou all

%% 1) Atitude do perseguidor e modo de operação

[t_att,  angEuler] = alinhar_tempo(t, attd_angEuler_chaser);
[t_modo, modo_op]  = alinhar_tempo(t, modo_operacao);

if plot_sel == 0 || plot_sel == 1
%% 1a) Ângulos de Euler (completo + ZOOM)  [FIGURE 1]
t_zoom_ini = 400;  % início do zoom [s]

fig1a = figure;
set(fig1a,'DefaultAxesFontSize',   fs);
set(fig1a,'DefaultTextFontSize',   fs);
set(fig1a,'DefaultLegendFontSize', fs);

%% Layout: agora 3 subplots (completo, zoom 400+, zoom 0-15 s)
t_zoom_ini = 400;      % zoom longo (igual ao que você já usa)
t_zoom_ini2 = 0;       % zoom inicial
t_zoom_fim2 = 10;      % zoom inicial até 15 s

tlo1a = tiledlayout(fig1a, 3, 1, 'TileSpacing','compact', 'Padding','compact');

%% =================== (1a-1) Euler (completo) ===================
ax1 = nexttile(tlo1a); hold(ax1,'on');

h_roll  = plot(ax1, t_att, angEuler(:,1), 'k', 'LineWidth', 2);
h_pitch = plot(ax1, t_att, angEuler(:,2), 'r', 'LineWidth', 2);
h_yaw   = plot(ax1, t_att, angEuler(:,3), 'b', 'LineWidth', 2);

axes(ax1);
h_sw = vline_mr(t_switch, lw_mr, mr_style);
h_ac = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(ax1,'off');
grid(ax1,'on');
xlabel(ax1,'Tempo [s]');
% ylabel(ax1,'Atitude do perseguidor [grau]');
xlim(ax1,[0 tempo_final]);

leg1a = legend(ax1, [h_roll, h_pitch, h_yaw, h_sw, h_ac], ...
    {'Ângulo de Euler (X, roll)', ...
     'Ângulo de Euler (Y, pitch)', ...
     'Ângulo de Euler (Z, yaw)', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');

%% =================== (1a-2) Euler (ZOOM de 400 s até o final) ===================
ax2 = nexttile(tlo1a); hold(ax2,'on');

plot(ax2, t_att, angEuler(:,1), 'k', 'LineWidth', 2);
plot(ax2, t_att, angEuler(:,2), 'r', 'LineWidth', 2);
plot(ax2, t_att, angEuler(:,3), 'b', 'LineWidth', 2);

axes(ax2);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);

if ~isnan(t_fim_missao)
    xline(t_fim_missao, 'm-.', 'LineWidth', 2);
end

hold(ax2,'off');
grid(ax2,'on');
xlabel(ax2,'Tempo [s]');
ylabel(ax2,'Atitude do perseguidor [grau]');
xlim(ax2,[t_zoom_ini tempo_final]);

%% =================== (1a-3) Euler (ZOOM de 0 a 15 s) ===================
ax3 = nexttile(tlo1a); hold(ax3,'on');

plot(ax3, t_att, angEuler(:,1), 'k', 'LineWidth', 2);
plot(ax3, t_att, angEuler(:,2), 'r', 'LineWidth', 2);
plot(ax3, t_att, angEuler(:,3), 'b', 'LineWidth', 2);

axes(ax3);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);

if ~isnan(t_fim_missao)
    xline(t_fim_missao, 'm-.', 'LineWidth', 2);
end

hold(ax3,'off');
grid(ax3,'on');
xlabel(ax3,'Tempo [s]');
% ylabel(ax3,'Atitude do perseguidor [grau]');
xlim(ax3,[t_zoom_ini2 t_zoom_fim2]);

% fonte
set([ax1 ax2 ax3],'FontSize',fs);
set(leg1a,'FontSize',fs);
set(findall(fig1a,'Type','Text'),'FontSize',fs);



%% =================== 1b) Quaternion  [FIGURE 2] ===================

fig1b = figure;
set(fig1b,'DefaultAxesFontSize',   fs);
set(fig1b,'DefaultTextFontSize',   fs);
set(fig1b,'DefaultLegendFontSize', fs);

ax3 = axes(fig1b); hold(ax3,'on');

% ===== dados do quaternion =====
[t_qatt, q_ch] = alinhar_tempo(t, attde_quaternion_chaser);

hq1 = plot(ax3, t_qatt, q_ch(:,1), 'k', 'LineWidth', 2);
hq2 = plot(ax3, t_qatt, q_ch(:,2), 'r', 'LineWidth', 2);
hq3 = plot(ax3, t_qatt, q_ch(:,3), 'b', 'LineWidth', 2);
hq4 = plot(ax3, t_qatt, q_ch(:,4), 'm', 'LineWidth', 2);

axes(ax3);
h_sw_q = vline_mr(t_switch, lw_mr, mr_style);
h_ac_q = vline_acoplamento(t_acoplamento, lw_ac, ac_style);


hold(ax3,'off');
grid(ax3,'on');
xlabel(ax3,'Tempo [s]');
ylabel(ax3,'Atitude do perseguidor (quaternion)');
xlim(ax3,[0 tempo_final]);
ylim(ax3, [-1 1.2]);

leg1b = legend(ax3, [hq1 hq2 hq3 hq4 h_sw_q h_ac_q], ...
    {'Quaternion q1 (w)', ...
     'Quaternion q2 (x)', ...
     'Quaternion q3 (y)', ...
     'Quaternion q4 (z)', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento',}, ...
    'Location','eastoutside','Interpreter','none');

set(ax3,'FontSize',fs);
set(leg1b,'FontSize',fs);
set(findall(fig1b,'Type','Text'),'FontSize',fs);

    % =================== 1c) Modo de operação ===================
    % fig1c = figure;
    % set(fig1c,'DefaultAxesFontSize',   fs);
    % set(fig1c,'DefaultTextFontSize',   fs);
    % set(fig1c,'DefaultLegendFontSize', fs);
    %
    % hold on;
    % h_modo = stairs(t_modo, modo_op, 'k', 'LineWidth', 2);
    % h_sw_m = vline_mr(t_switch, lw_mr, mr_style);
    % h_ac_m = vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    % hold off;
    %
    % grid on;
    % xlabel('Tempo [s]');
    % ylabel('Modo de operação [-]');
    % yticks([0 1]);
    % ylim([-0.2 1.2]);
    % xlim([0 tempo_final]);
    %
    % leg1c = legend([h_modo, h_sw_m, h_ac_m], ...
    %     {'Modo de operação [-]', ...
    %      'Início do modo operacional do MR', ...
    %      'Instante de acoplamento'}, ...
    %     'Location','eastoutside','Interpreter','none');
    %
    % set(gca,'FontSize',fs);
    % set(leg1c,'FontSize',fs);
    % set(findall(fig1b,'Type','Text'),'FontSize',fs);

end

if deve_imprimir(print_sel, 1)
    imprimir_resumo_vetor('Bloco 1 - ângulos de Euler', t_att, angEuler, {'roll','pitch','yaw'});
    imprimir_resumo_vetor('Bloco 1 - modo de operação', t_modo, modo_op, {'modo'});
end


%% 2) Dinâmica do perseguidor: velocidade e aceleração angular (Euler + Lagrange)
% -------------------------------------------------------------------------

if plot_sel == 0 || plot_sel == 2

    % ===== sinais =====
    [t_w,  w_base]  = alinhar_tempo(t, dinEulerLagrange_veloc_angular_viaIntegral);
    [t_dw, dw_base] = alinhar_tempo(t, dinEulerLagrange_acel_angular_viaIntegral);

    lim_vel = [-1.5 1.5];   % ajuste se quiser
    lim_acc = [-5 5];

    %% ===================== FIG 2W: VELOCIDADE ANGULAR (4 subplots) =====================
    fig2w = figure;
    set(fig2w,'DefaultAxesFontSize',   fs);
    set(fig2w,'DefaultTextFontSize',   fs);
    set(fig2w,'DefaultLegendFontSize', fs);

    tlo2w = tiledlayout(fig2w, 4, 1, 'TileSpacing','compact', 'Padding','compact');

    % ------------------ (2W-1) 3 eixos juntos ------------------
    axW0 = nexttile(tlo2w); hold(axW0,'on');
    hwx = plot(axW0, t_w, w_base(:,1), 'k', 'LineWidth', 1.5);
    hwy = plot(axW0, t_w, w_base(:,2), 'r', 'LineWidth', 1.5);
    hwz = plot(axW0, t_w, w_base(:,3), 'b', 'LineWidth', 1.5);

    hsw0 = vline_mr(t_switch, lw_mr, mr_style);
    hac0 = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

    hold(axW0,'off');
    grid(axW0,'on');
    xlabel(axW0,'Tempo [s]');
    xlim(axW0,[0 tempo_final]);
    ylim([-0.1, 0.1]);

    legW = legend(axW0, [hwx hwy hwz hsw0 hac0], ...
        {'Velocidade angular (x) [rad/s]', ...
         'Velocidade angular (y) [rad/s]', ...
         'Velocidade angular (z) [rad/s]', ...
         'Início do modo operacional do MR', ...
         'Instante de acoplamento'}, ...
        'Location','eastoutside','Interpreter','none');

    % ------------------ (2W-2) eixo x ------------------
    axWx = nexttile(tlo2w); hold(axWx,'on');
    plot(axWx, t_w, w_base(:,1), 'k', 'LineWidth', 1.5);   % x = preto (igual ao 1º)
    vline_mr(t_switch, lw_mr, mr_style);
    vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(axWx,'off');
    grid(axWx,'on');
    xlabel(axWx,'Tempo [s]');
    xlim(axWx,[0 tempo_final]);
    ylim([-0.1, 0.1]);
    
    % ------------------ (2W-3) eixo y ------------------
    axWy = nexttile(tlo2w); hold(axWy,'on');
    plot(axWy, t_w, w_base(:,2), 'r', 'LineWidth', 1.5);   % y = vermelho (igual ao 1º)
    vline_mr(t_switch, lw_mr, mr_style);
    vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(axWy,'off');
    grid(axWy,'on');
    xlabel(axWy,'Tempo [s]');
    xlim(axWy,[0 tempo_final]);
    ylim([-0.1, 0.1]);
    ylabel(axWy,'Velocidade angular do perseguidor [rad/s]');  % ylabel único (ok)
    
    % ------------------ (2W-4) eixo z ------------------
    axWz = nexttile(tlo2w); hold(axWz,'on');
    plot(axWz, t_w, w_base(:,3), 'b', 'LineWidth', 1.5);   % z = azul (igual ao 1º)
    vline_mr(t_switch, lw_mr, mr_style);
    vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(axWz,'off');
    grid(axWz,'on');
    xlabel(axWz,'Tempo [s]');
    xlim(axWz,[0 tempo_final]);
    ylim([-0.1, 0.1]);

    linkaxes([axW0 axWx axWy axWz],'x');
    set([axW0 axWx axWy axWz],'FontSize',fs);
    set(legW,'FontSize',fs);
    set(findall(fig2w,'Type','Text'),'FontSize',fs);



%% ===================== ACELERAÇÃO ANGULAR: FIGURE 1 (2 subplots: completo + zoom 0-10 s) =====================

fig2dw_a = figure;
set(fig2dw_a,'DefaultAxesFontSize',   fs);
set(fig2dw_a,'DefaultTextFontSize',   fs);
set(fig2dw_a,'DefaultTextFontSize',   fs);
set(fig2dw_a,'DefaultLegendFontSize', fs);

tlo2dw_a = tiledlayout(fig2dw_a, 2, 1, 'TileSpacing','compact', 'Padding','compact');

% ------------------ (2dW-A1) 3 eixos juntos (completo) ------------------
axDW0 = nexttile(tlo2dw_a); hold(axDW0,'on');
hdwx = plot(axDW0, t_dw, dw_base(:,1), 'k', 'LineWidth', 1.5);
hdwy = plot(axDW0, t_dw, dw_base(:,2), 'r', 'LineWidth', 1.5);
hdwz = plot(axDW0, t_dw, dw_base(:,3), 'b', 'LineWidth', 1.5);

axes(axDW0);
hswDW0 = vline_mr(t_switch, lw_mr, mr_style);
hacDW0 = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(axDW0,'off');
grid(axDW0,'on');
xlabel(axDW0,'Tempo [s]');
xlim(axDW0,[0 tempo_final]);
ylim([-3, 3]);
ylabel('Aceleração angular [rad/s^2]');

legDW_a = legend(axDW0, [hdwx hdwy hdwz hswDW0 hacDW0], ...
    {'Aceleração angular (x) [rad/s^2]', ...
     'Aceleração angular (y) [rad/s^2]', ...
     'Aceleração angular (z) [rad/s^2]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');

% ------------------ (2dW-A2) 3 eixos juntos (ZOOM 0-10 s) ------------------
axDWz0 = nexttile(tlo2dw_a); hold(axDWz0,'on');
plot(axDWz0, t_dw, dw_base(:,1), 'k', 'LineWidth', 1.5);
plot(axDWz0, t_dw, dw_base(:,2), 'r', 'LineWidth', 1.5);
plot(axDWz0, t_dw, dw_base(:,3), 'b', 'LineWidth', 1.5);

axes(axDWz0);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(axDWz0,'off');
grid(axDWz0,'on');
xlabel(axDWz0,'Tempo [s]');
xlim(axDWz0,[0 10]);
ylim([-125, 5]);

set([axDW0 axDWz0],'FontSize',fs);
set(legDW_a,'FontSize',fs);
set(findall(fig2dw_a,'Type','Text'),'FontSize',fs);


%% ===================== ACELERAÇÃO ANGULAR: FIGURE 2 (3 subplots: x, y, z) =====================

fig2dw_b = figure;
set(fig2dw_b,'DefaultAxesFontSize',   fs);
set(fig2dw_b,'DefaultTextFontSize',   fs);
set(fig2dw_b,'DefaultLegendFontSize', fs);

tlo2dw_b = tiledlayout(fig2dw_b, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% ------------------ (2dW-B1) eixo x ------------------
axDWx = nexttile(tlo2dw_b); hold(axDWx,'on');
plot(axDWx, t_dw, dw_base(:,1), 'k', 'LineWidth', 1.5);

axes(axDWx);
hswDWx = vline_mr(t_switch, lw_mr, mr_style);
hacDWx = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(axDWx,'off');
grid(axDWx,'on');
xlabel(axDWx,'Tempo [s]');
xlim(axDWx,[0 tempo_final]);
ylim([-3, 3]);

% ------------------ (2dW-B2) eixo y (com ylabel + legenda) ------------------
axDWy = nexttile(tlo2dw_b); hold(axDWy,'on');
h_y = plot(axDWy, t_dw, dw_base(:,2), 'r', 'LineWidth', 1.5);

axes(axDWy);
hswDWy = vline_mr(t_switch, lw_mr, mr_style);
hacDWy = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

% dummies para a legenda ficar completa no subplot do meio
h_x_dummy = plot(axDWy, NaN, NaN, 'k', 'LineWidth', 1.5);
h_z_dummy = plot(axDWy, NaN, NaN, 'b', 'LineWidth', 1.5);

hold(axDWy,'off');
grid(axDWy,'on');
xlabel(axDWy,'Tempo [s]');
xlim(axDWy,[0 tempo_final]);
ylim([-3, 3]);
ylabel('Aceleração angular do perseguidor [rad/s^2]');

legend(axDWy, [h_x_dummy h_y h_z_dummy hswDWy hacDWy], ...
    {'Aceleração angular (x) [rad/s^2]', ...
     'Aceleração angular (y) [rad/s^2]', ...
     'Aceleração angular (z) [rad/s^2]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');

% ------------------ (2dW-B3) eixo z ------------------
axDWz = nexttile(tlo2dw_b); hold(axDWz,'on');
plot(axDWz, t_dw, dw_base(:,3), 'b', 'LineWidth', 1.5);

axes(axDWz);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(axDWz,'off');
grid(axDWz,'on');
xlabel(axDWz,'Tempo [s]');
xlim(axDWz,[0 tempo_final]);
ylim([-3, 3]);

% mesma escala de tempo nos 3 eixos
linkaxes([axDWx axDWy axDWz],'x');

set([axDWx axDWy axDWz],'FontSize',fs);
set(findall(fig2dw_b,'Type','Text'),'FontSize',fs);

end




if deve_imprimir(print_sel, 2)
    imprimir_resumo_vetor('Bloco 2 - velocidade angular base (Euler+Lagrange)', ...
                          t_w, w_base, {'x','y','z'});
    imprimir_resumo_vetor('Bloco 2 - aceleração angular base (Euler+Lagrange)', ...
                          t_dw, dw_base, {'x','y','z'});
end


%% 3) Controle de atitude (torque PD, torque compensado e reação do perseguidor)
% -------------------------------------------------------------------------
if plot_sel == 0 || plot_sel == 3

    [t_PD_att,  tau_PD_att]  = alinhar_tempo(t, PD_torque_controle_attde);
    [t_PD_comp, tau_PD_comp] = alinhar_tempo(t, PD_torque_controle_compensado);
    [t_tr_base, tau_reacao]  = alinhar_tempo(t, torque_reacao_base);

 %% ===================== 3a: torque PD (3 subplots, legenda ao lado do subplot 2) =====================
fig3a_pd = figure;
set(fig3a_pd,'DefaultAxesFontSize',   fs);
set(fig3a_pd,'DefaultTextFontSize',   fs);
set(fig3a_pd,'DefaultLegendFontSize', fs);

tlo3a_pd = tiledlayout(fig3a_pd, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% ---------- eixo X (roll) ----------
ax_pd_x = nexttile(tlo3a_pd); hold(ax_pd_x,'on');
plot(ax_pd_x, t_PD_att, tau_PD_att(:,1), 'k', 'LineWidth', 1.5);
axes(ax_pd_x); vline_mr(t_switch, lw_mr, mr_style);
axes(ax_pd_x); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax_pd_x,'off');
grid(ax_pd_x,'on');
xlabel(ax_pd_x,'Tempo [s]');
xlim(ax_pd_x,[0 tempo_final]);
ylim([-1 1]);

% ---------- eixo Y (pitch) ----------
ax_pd_y = nexttile(tlo3a_pd); hold(ax_pd_y,'on');
plot(ax_pd_y, t_PD_att, tau_PD_att(:,2), 'r', 'LineWidth', 1.5);
axes(ax_pd_y); hsw_pd = vline_mr(t_switch, lw_mr, mr_style);                
axes(ax_pd_y); hac_pd = vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax_pd_y,'off');
grid(ax_pd_y,'on');
xlabel(ax_pd_y,'Tempo [s]');
ylabel('Torque do controlador de atitude [Nm]');
xlim(ax_pd_y,[0 tempo_final]);
ylim([-1 1]);


% ---------- eixo Z (yaw) ----------
ax_pd_z = nexttile(tlo3a_pd); hold(ax_pd_z,'on');
plot(ax_pd_z, t_PD_att, tau_PD_att(:,3), 'b', 'LineWidth', 1.5);
axes(ax_pd_z); vline_mr(t_switch, lw_mr, mr_style);
axes(ax_pd_z); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax_pd_z,'off');
grid(ax_pd_z,'on');
xlabel(ax_pd_z,'Tempo [s]');
xlim(ax_pd_z,[0 tempo_final]);
ylim([-1 1]);


% legenda com X/Y/Z + MR + acoplamento (TODOS HANDLES DO ax_pd_y)
hold(ax_pd_y,'on');
hpd_x_dummy = plot(ax_pd_y, NaN, NaN, 'k', 'LineWidth', 1.5);
hpd_y_dummy = plot(ax_pd_y, NaN, NaN, 'r', 'LineWidth', 1.5);
hpd_z_dummy = plot(ax_pd_y, NaN, NaN, 'b', 'LineWidth', 1.5);
hold(ax_pd_y,'off');

legend(ax_pd_y, [hpd_x_dummy, hpd_y_dummy, hpd_z_dummy, hsw_pd, hac_pd], ...
    {'Torque de controle (X, roll) [Nm]', ...
     'Torque de controle (Y, pitch) [Nm]', ...
     'Torque de controle (Z, yaw) [Nm]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside', 'Interpreter','none');

linkaxes([ax_pd_x ax_pd_y ax_pd_z],'x');


%% ===================== 3b: torque de reação da espaçonave (3 subplots, legenda ao lado do subplot 2) =====================
fig3a_reac = figure;
set(fig3a_reac,'DefaultAxesFontSize',   fs);
set(fig3a_reac,'DefaultTextFontSize',   fs);
set(fig3a_reac,'DefaultLegendFontSize', fs);

tlo3a_reac = tiledlayout(fig3a_reac, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% ---------- eixo X (roll) ----------
ax_r_x = nexttile(tlo3a_reac); hold(ax_r_x,'on');
plot(ax_r_x, t_tr_base, tau_reacao(:,1), 'k', 'LineWidth', 1.5);
axes(ax_r_x); vline_mr(t_switch, lw_mr, mr_style);
axes(ax_r_x); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax_r_x,'off');
grid(ax_r_x,'on');
xlabel(ax_r_x,'Tempo [s]');
xlim(ax_r_x,[0 tempo_final]);
ylim(ax_r_x,[-1 1]);

% ---------- eixo Y (pitch) ----------
ax_r_y = nexttile(tlo3a_reac); hold(ax_r_y,'on');
plot(ax_r_y, t_tr_base, tau_reacao(:,2), 'r', 'LineWidth', 1.5);
axes(ax_r_y); hsw_r = vline_mr(t_switch, lw_mr, mr_style);                 
axes(ax_r_y); hac_r = vline_acoplamento(t_acoplamento, lw_ac, ac_style);   
hold(ax_r_y,'off');
grid(ax_r_y,'on');
xlabel(ax_r_y,'Tempo [s]');
xlim(ax_r_y,[0 tempo_final]);
ylim(ax_r_y,[-1 1]);
ylabel('Torque de reação da espaçonave [Nm]');

% ---------- eixo Z (yaw) ----------
ax_r_z = nexttile(tlo3a_reac); hold(ax_r_z,'on');
plot(ax_r_z, t_tr_base, tau_reacao(:,3), 'b', 'LineWidth', 1.5);
axes(ax_r_z); vline_mr(t_switch, lw_mr, mr_style);
axes(ax_r_z); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax_r_z,'off');
grid(ax_r_z,'on');
xlabel(ax_r_z,'Tempo [s]');
xlim(ax_r_z,[0 tempo_final]);
ylim(ax_r_z,[-1 1]);

% legenda com X/Y/Z + MR + acoplamento (TODOS HANDLES DO ax_r_y)
hold(ax_r_y,'on');
hr_x_dummy = plot(ax_r_y, NaN, NaN, 'k', 'LineWidth', 1.5);
hr_y_dummy = plot(ax_r_y, NaN, NaN, 'r', 'LineWidth', 1.5);
hr_z_dummy = plot(ax_r_y, NaN, NaN, 'b', 'LineWidth', 1.5);
hold(ax_r_y,'off');

legend(ax_r_y, [hr_x_dummy, hr_y_dummy, hr_z_dummy, hsw_r, hac_r], ...
    {'Torque de reação da espaçonave (X, roll) [Nm]', ...
     'Torque de reação da espaçonave (Y, pitch) [Nm]', ...
     'Torque de reação da espaçonave (Z, yaw) [Nm]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside', 'Interpreter','none');

linkaxes([ax_r_x ax_r_y ax_r_z],'x');


  %% ===================== 3c: torque compensado em 3 subplots =====================
fig3b = figure;
set(fig3b,'DefaultAxesFontSize',   fs);
set(fig3b,'DefaultTextFontSize',   fs);
set(fig3b,'DefaultLegendFontSize', fs);

tlo3b = tiledlayout(fig3b, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% ---------- subplot x ----------
ax2x = nexttile(tlo3b); hold(ax2x,'on');
plot(ax2x, t_PD_comp, tau_PD_comp(:,1), 'k', 'LineWidth', 1.5);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax2x,'off');
grid(ax2x,'on');
xlabel(ax2x,'Tempo [s]');
xlim(ax2x,[0 tempo_final]);
ylim(ax2x,[-2 2]);

% ---------- subplot y ----------
ax2y = nexttile(tlo3b); hold(ax2y,'on');

% sinais reais
hy = plot(ax2y, t_PD_comp, tau_PD_comp(:,2), 'r', 'LineWidth', 1.5);

% linhas verticais (reais, neste eixo)
hsw = vline_mr(t_switch, lw_mr, mr_style);
hac = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

% "dummies" para X e Z (para aparecerem na legenda do subplot 2)
hx_dummy = plot(ax2y, NaN, NaN, 'k', 'LineWidth', 1.5);
hz_dummy = plot(ax2y, NaN, NaN, 'b', 'LineWidth', 1.5);

hold(ax2y,'off');
grid(ax2y,'on');
xlabel(ax2y,'Tempo [s]');
xlim(ax2y,[0 tempo_final]);
ylim(ax2y,[-2 2]);
ylabel('Torque compensado [Nm]');

legend(ax2y, [hx_dummy, hy, hz_dummy, hsw, hac], ...
    {'Torque de controle (compensado) (X, roll) [Nm]', ...
     'Torque de controle (compensado) (Y, pitch) [Nm]', ...
     'Torque de controle (compensado) (Z, yaw) [Nm]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside', 'Interpreter','none');


% ---------- subplot z ----------
ax2z = nexttile(tlo3b); hold(ax2z,'on');
plot(ax2z, t_PD_comp, tau_PD_comp(:,3), 'b', 'LineWidth', 1.5);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax2z,'off');
grid(ax2z,'on');
xlabel(ax2z,'Tempo [s]');
xlim(ax2z,[0 tempo_final]);
ylim(ax2z,[-2 2]);

% mesma escala de tempo nos 3 eixos
linkaxes([ax2x ax2y ax2z],'x');



end

if deve_imprimir(print_sel, 3)
    imprimir_resumo_vetor('Bloco 3 - torque PD atitude', ...
        t_PD_att, tau_PD_att, {'x','y','z'});
    imprimir_resumo_vetor('Bloco 3 - torque compensado atitude', ...
        t_PD_comp, tau_PD_comp, {'x','y','z'});
    imprimir_resumo_vetor('Bloco 3 - torque de reação na base', ...
        t_tr_base, tau_reacao, {'x','y','z'});
end


%% 4) Posição, velocidade e torque das juntas do MR 
% -------------------------------------------------------------------------
if plot_sel == 0 || plot_sel == 4

    [t_q,   q_j]   = alinhar_tempo(t, dinLagrange_posicao_juntas);
    [t_dq,  dq_j]  = alinhar_tempo(t, dinLagrange_veloc_juntas);
    [t_tau, tau_j] = alinhar_tempo(t, juntas_torque_controle);

    %% =================== 4a: POSIÇÃO (rad e deg) ===================
    fig4a = figure;
    set(fig4a,'DefaultAxesFontSize',   fs);
    set(fig4a,'DefaultTextFontSize',   fs);
    set(fig4a,'DefaultLegendFontSize', fs);

    tlo4a = tiledlayout(fig4a, 2, 1, 'TileSpacing','compact', 'Padding','compact');

    % ---- (4A-1) posição em rad ----
    ax4a1 = nexttile(tlo4a); hold(ax4a1,'on');
    hq1r = plot(ax4a1, t_q, q_j(:,1), 'k', 'LineWidth', 1.5);
    hq2r = plot(ax4a1, t_q, q_j(:,2), 'r', 'LineWidth', 1.5);
    hq3r = plot(ax4a1, t_q, q_j(:,3), 'b', 'LineWidth', 1.5);
    hsw_r = vline_mr(t_switch, lw_mr, mr_style);
    hac_r = vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(ax4a1,'off');

    grid(ax4a1,'on');
    xlabel(ax4a1,'Tempo [s]');
    ylabel(ax4a1,'Posição das juntas [rad]');
    xlim(ax4a1,[0 tempo_final]);
    ylim([-pi pi]);
    leg4a1 = legend(ax4a1, [hq1r hq2r hq3r hsw_r hac_r], ...
        {'Posição da junta 1 [rad]', ...
         'Posição da junta 2 [rad]', ...
         'Posição da junta 3 [rad]', ...
         'Início do modo operacional do MR', ...
         'Instante de acoplamento'}, ...
        'Location','eastoutside','Interpreter','none');

    % ---- (4A-2) posição em deg ----
    ax4a2 = nexttile(tlo4a); hold(ax4a2,'on');
    q_deg = rad2deg(q_j);

    hq1d = plot(ax4a2, t_q, q_deg(:,1), 'k', 'LineWidth', 1.5);
    hq2d = plot(ax4a2, t_q, q_deg(:,2), 'r', 'LineWidth', 1.5);
    hq3d = plot(ax4a2, t_q, q_deg(:,3), 'b', 'LineWidth', 1.5);
    vline_mr(t_switch, lw_mr, mr_style);
    vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(ax4a2,'off');

    grid(ax4a2,'on');
    xlabel(ax4a2,'Tempo [s]');
    ylabel(ax4a2,'Posição das juntas [deg]');
    xlim(ax4a2,[0 tempo_final]);
    ylim([-180 180]);


    leg4a2 = legend(ax4a2, [hq1d hq2d hq3d hsw_r hac_r], ...
        {'Posição da junta 1 [deg]', ...
         'Posição da junta 2 [deg]', ...
         'Posição da junta 3 [deg]', ...
         'Início do modo operacional do MR', ...
         'Instante de acoplamento'}, ...
        'Location','eastoutside','Interpreter','none');

    linkaxes([ax4a1 ax4a2],'x');
    set([ax4a1 ax4a2],'FontSize',fs);
    set([leg4a1 leg4a2],'FontSize',fs);


    % ------------------- Resumos numéricos (TOTAL) -------------------
    imprimir_resumo_vetor('Bloco 4a - posição das juntas [rad]', t_q,  q_j,   {'junta 1','junta 2','junta 3'});
    imprimir_resumo_vetor('Bloco 4a - posição das juntas [deg]', t_q,  q_deg, {'junta 1','junta 2','junta 3'});
    imprimir_resumo_vetor('Bloco 4b - velocidade das juntas [rad/s]', t_dq, dq_j,  {'junta 1','junta 2','junta 3'});
    imprimir_resumo_vetor('Bloco 4c - torque nas juntas [Nm]', t_tau, tau_j, {'junta 1','junta 2','junta 3'});

    % ------------------- Amostras em instantes de interesse -------------------
    fprintf('\n[Bloco 4] Amostras pontuais (amostra finita mais próxima no tempo)\n');

    % t_switch e t_acoplamento já existem no script; aqui só usa.
    Tq = [t_q(1); t_switch; t_acoplamento; tempo_final];
    Tn = {'t_ini','t_switch','t_acoplamento','tempo_final'};

    % função anônima local (não depende de nada externo)
    amostra_mais_proxima = @(tt,YY,tref) local_amostra(tt,YY,tref);

    % POSIÇÃO [rad]
    for i = 1:numel(Tq)
        if ~isfinite(Tq(i)), continue; end
        [ti, yi] = amostra_mais_proxima(t_q, q_j, Tq(i));
        fprintf('  q_j   @ %-13s: t = %9.3f s | [%.6g  %.6g  %.6g] rad\n', ...
                Tn{i}, ti, yi(1), yi(2), yi(3));
    end

    % POSIÇÃO [deg]
    for i = 1:numel(Tq)
        if ~isfinite(Tq(i)), continue; end
        [ti, yi] = amostra_mais_proxima(t_q, q_deg, Tq(i));
        fprintf('  q_deg @ %-13s: t = %9.3f s | [%.6g  %.6g  %.6g] deg\n', ...
                Tn{i}, ti, yi(1), yi(2), yi(3));
    end

    % VELOCIDADE [rad/s]
    for i = 1:numel(Tq)
        if ~isfinite(Tq(i)), continue; end
        [ti, yi] = amostra_mais_proxima(t_dq, dq_j, Tq(i));
        fprintf('  dq_j  @ %-13s: t = %9.3f s | [%.6g  %.6g  %.6g] rad/s\n', ...
                Tn{i}, ti, yi(1), yi(2), yi(3));
    end

    % TORQUE [Nm]
    for i = 1:numel(Tq)
        if ~isfinite(Tq(i)), continue; end
        [ti, yi] = amostra_mais_proxima(t_tau, tau_j, Tq(i));
        fprintf('  tau_j @ %-13s: t = %9.3f s | [%.6g  %.6g  %.6g] Nm\n', ...
                Tn{i}, ti, yi(1), yi(2), yi(3));
    end




    %% =================== 4b: VELOCIDADE (3 subplots) ===================
fig4b = figure;
set(fig4b,'DefaultAxesFontSize',   fs);
set(fig4b,'DefaultTextFontSize',   fs);
set(fig4b,'DefaultLegendFontSize', fs);

tlo4b = tiledlayout(fig4b, 3, 1, 'TileSpacing','compact', 'Padding','compact');

% (4B-1) junta 1
ax4b1 = nexttile(tlo4b); hold(ax4b1,'on');
plot(ax4b1, t_dq, dq_j(:,1), 'k', 'LineWidth', 1.5);
axes(ax4b1); vline_mr(t_switch, lw_mr, mr_style);
axes(ax4b1); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax4b1,'off');
grid(ax4b1,'on');
xlabel(ax4b1,'Tempo [s]');
xlim(ax4b1,[0 tempo_final]);
ylim([-0.3 0.3]);

% (4B-2) junta 2  
ax4b2 = nexttile(tlo4b); hold(ax4b2,'on');
plot(ax4b2, t_dq, dq_j(:,2), 'r', 'LineWidth', 1.5);
axes(ax4b2); hsw_v = vline_mr(t_switch, lw_mr, mr_style);              
axes(ax4b2); hac_v = vline_acoplamento(t_acoplamento, lw_ac, ac_style);  
hold(ax4b2,'off');
grid(ax4b2,'on');
xlabel(ax4b2,'Tempo [s]');
xlim(ax4b2,[0 tempo_final]);
ylabel('Velocidade das juntas [rad/s]');  
ylim([-0.3 0.3]);

% (4B-3) junta 3
ax4b3 = nexttile(tlo4b); hold(ax4b3,'on');
plot(ax4b3, t_dq, dq_j(:,3), 'b', 'LineWidth', 1.5);
axes(ax4b3); vline_mr(t_switch, lw_mr, mr_style);
axes(ax4b3); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
hold(ax4b3,'off');
grid(ax4b3,'on');
xlabel(ax4b3,'Tempo [s]');
xlim(ax4b3,[0 tempo_final]);
ylim([-0.3 0.3]);

% legenda: 3 juntas + MR + acoplamento (todos handles pertencem ao ax4b2)
hold(ax4b2,'on');
hv1_dummy = plot(ax4b2, NaN, NaN, 'k', 'LineWidth', 1.5);
hv2_dummy = plot(ax4b2, NaN, NaN, 'r', 'LineWidth', 1.5);
hv3_dummy = plot(ax4b2, NaN, NaN, 'b', 'LineWidth', 1.5);
hold(ax4b2,'off');

legend(ax4b2, [hv1_dummy hv2_dummy hv3_dummy hsw_v hac_v], ...
    {'Velocidade da junta 1 [rad/s]', ...
     'Velocidade da junta 2 [rad/s]', ...
     'Velocidade da junta 3 [rad/s]', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');

linkaxes([ax4b1 ax4b2 ax4b3],'x');
set([ax4b1 ax4b2 ax4b3],'FontSize',fs);


    %% =================== 4c: TORQUE (3 subplots) ===================
    fig4c = figure;
    set(fig4c,'DefaultAxesFontSize',   fs);
    set(fig4c,'DefaultTextFontSize',   fs);
    set(fig4c,'DefaultLegendFontSize', fs);
    
    tlo4c = tiledlayout(fig4c, 3, 1, 'TileSpacing','compact', 'Padding','compact');
    
    % (4C-1) junta 1
    ax4c1 = nexttile(tlo4c); hold(ax4c1,'on');
    plot(ax4c1, t_tau, tau_j(:,1), 'k', 'LineWidth', 1.5);
    axes(ax4c1); vline_mr(t_switch, lw_mr, mr_style);
    axes(ax4c1); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(ax4c1,'off');
    grid(ax4c1,'on');
    xlabel(ax4c1,'Tempo [s]');
    xlim(ax4c1,[0 tempo_final]);
    ylim([-0.3 0.3]);
    
    % (4C-2) junta 2 
    ax4c2 = nexttile(tlo4c); hold(ax4c2,'on');
    plot(ax4c2, t_tau, tau_j(:,2), 'r', 'LineWidth', 1.5);
    axes(ax4c2); hsw_t = vline_mr(t_switch, lw_mr, mr_style);                
    axes(ax4c2); hac_t = vline_acoplamento(t_acoplamento, lw_ac, ac_style); 
    hold(ax4c2,'off');
    grid(ax4c2,'on');
    xlabel(ax4c2,'Tempo [s]');
    xlim(ax4c2,[0 tempo_final]);
    ylim([-0.3 0.3]);
    ylabel('Torque nas juntas [Nm]'); 
    
    % (4C-3) junta 3
    ax4c3 = nexttile(tlo4c); hold(ax4c3,'on');
    plot(ax4c3, t_tau, tau_j(:,3), 'b', 'LineWidth', 1.5);
    axes(ax4c3); vline_mr(t_switch, lw_mr, mr_style);
    axes(ax4c3); vline_acoplamento(t_acoplamento, lw_ac, ac_style);
    hold(ax4c3,'off');
    grid(ax4c3,'on');
    xlabel(ax4c3,'Tempo [s]');
    xlim(ax4c3,[0 tempo_final]);
    ylim([-0.3 0.3]);

    
    % legenda: 3 juntas + MR + acoplamento (todos handles pertencem ao ax4c2)
    hold(ax4c2,'on');
    ht1_dummy = plot(ax4c2, NaN, NaN, 'k', 'LineWidth', 1.5);
    ht2_dummy = plot(ax4c2, NaN, NaN, 'r', 'LineWidth', 1.5);
    ht3_dummy = plot(ax4c2, NaN, NaN, 'b', 'LineWidth', 1.5);
    hold(ax4c2,'off');
    
    legend(ax4c2, [ht1_dummy ht2_dummy ht3_dummy hsw_t hac_t], ...
        {'Torque na junta 1 [Nm]', ...
         'Torque na junta 2 [Nm]', ...
         'Torque na junta 3 [Nm]', ...
         'Início do modo operacional do MR', ...
         'Instante de acoplamento'}, ...
        'Location','eastoutside','Interpreter','none');
    
    linkaxes([ax4c1 ax4c2 ax4c3],'x');
    set([ax4c1 ax4c2 ax4c3],'FontSize',fs);

end

%% 5) Erros de alinhamento / docking (DSU)
% -------------------------------------------------------------------------
if plot_sel == 0 || plot_sel == 5

    % ====== sinais alinhados ======
    [t_ep_m,  epos_m]  = alinhar_tempo(t, dsu_alinhamento_erro_pos_ferramenta_m);   % [N x 3]
    [t_ep_cm, epos_cm] = alinhar_tempo(t, dsu_alinhamento_erro_pos_ferramenta_cm);  % [N x 3]

    [t_en_m,  enorm_m]  = alinhar_tempo(t, dsu_alinhamento_norma_erro_pos_ferramenta_m);   % [N x 1]
    [t_en_cm, enorm_cm] = alinhar_tempo(t, dsu_alinhamento_norma_erro_pos_ferramenta_cm);  % [N x 1]

    [t_cpos,  cpos_ok]  = alinhar_tempo(t, dsu_alinhamento_cond_pos_ok);            % [N x 1]
    [t_cang,  cang_ok]  = alinhar_tempo(t, dsu_alinhamento_cond_angulo_ok);         % [N x 1]
    [t_est,   est_seq]  = alinhar_tempo(t, dsu_alinhamento_estado_sequenciador);    % [N x 1]

    % garantir vetores coluna (caso venha Nx1)
    enorm_m  = enorm_m(:,1);
    enorm_cm = enorm_cm(:,1);
    cpos_ok  = cpos_ok(:,1);
    cang_ok  = cang_ok(:,1);
    est_seq  = est_seq(:,1);

%% FIG 5A: erro vetorial de posição da ferramenta (m) + NORMA (mesmo tempo) + ZOOM (2 subplots)
t_zoom_ini = 600;  % início do zoom [s]

% IMPORTANTE: a norma DEVE ser calculada a partir de epos_m para ter o MESMO tempo e a MESMA duração.
% (evita o problema de o sinal "norma" do Simulink terminar antes)
enorm_em_t_ep = vecnorm(epos_m, 2, 2);   % [N x 1], mesma base t_ep_m

fig5a = figure;
set(fig5a,'DefaultAxesFontSize',   fs);
set(fig5a,'DefaultTextFontSize',   fs);
set(fig5a,'DefaultLegendFontSize', fs);

tlo5a = tiledlayout(fig5a, 2, 1, 'TileSpacing','compact', 'Padding','compact');

%% =================== (5A-1) completo ===================
ax5a1 = nexttile(tlo5a); hold(ax5a1,'on');

h_norm_style = '-.';
h_norm_linewidth = 3;

h_ex   = plot(ax5a1, t_ep_m, epos_m(:,1), 'k',  'LineWidth', 1.5);
h_ey   = plot(ax5a1, t_ep_m, epos_m(:,2), 'r',  'LineWidth', 1.5);
h_ez   = plot(ax5a1, t_ep_m, epos_m(:,3), 'b',  'LineWidth', 1.5);
h_norm = plot(ax5a1, t_ep_m, enorm_em_t_ep, h_norm_style, 'LineWidth', h_norm_linewidth, 'MarkerSize', 6);

axes(ax5a1);
h_sw = vline_mr(t_switch, lw_mr, mr_style);
h_ac = vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(ax5a1,'off');
grid(ax5a1,'on');
xlabel(ax5a1,'Tempo [s]');
% ylabel(ax5a1,'Erro de posição da ferramenta [m]');
xlim(ax5a1,[0 tempo_final]);
ylim(ax5a1,[-1.1 1.1]);
% ylabel(ax5a1,'Erro de posição da ferramenta [m]');

leg5a = legend(ax5a1, [h_ex h_ey h_ez h_norm h_sw h_ac], ...
    {'Erro de posição do efetuador final (x, ref. 0)', ...
     'Erro de posição do efetuador final (y, ref. 0)', ...
     'Erro de posição do efetuador final (z, ref. 0)', ...
     'Norma do erro de posição da ferramenta', ...
     'Início do modo operacional do MR', ...
     'Instante de acoplamento'}, ...
    'Location','eastoutside','Interpreter','none');

%% =================== (5A-2) zoom 600 s até o final ===================
ax5a2 = nexttile(tlo5a); hold(ax5a2,'on');

plot(ax5a2, t_ep_m, epos_m(:,1), 'k',  'LineWidth', 1);
plot(ax5a2, t_ep_m, epos_m(:,2), 'r',  'LineWidth', 1);
plot(ax5a2, t_ep_m, epos_m(:,3), 'b',  'LineWidth', 1);
plot(ax5a2, t_ep_m, enorm_em_t_ep, h_norm_style, 'LineWidth', h_norm_linewidth);

axes(ax5a2);
vline_mr(t_switch, lw_mr, mr_style);
vline_acoplamento(t_acoplamento, lw_ac, ac_style);

hold(ax5a2,'off');
grid(ax5a2,'on');
xlabel(ax5a2,'Tempo [s]');
xlim(ax5a2,[t_zoom_ini tempo_final]);
ylim(ax5a2,[-0.05 0.05]);
ylabel(ax5a2,'Erro de posição do efetuador final [m]');

linkaxes([ax5a1 ax5a2],'x');

set([ax5a1 ax5a2],'FontSize',fs);
set(leg5a,'FontSize',fs);
set(findall(fig5a,'Type','Text'),'FontSize',fs);



    % [BLOCO 5] Resumo no console
    if deve_imprimir(print_sel, 5)
        imprimir_resumo_vetor('Bloco 5 - Alinhamento erro posição ferramenta [m]', ...
                              t_ep_m, epos_m, {'x','y','z'});
        imprimir_resumo_vetor('Bloco 5 - Norma alinhamento erro posição ferramenta [m]', ...
                              t_en_m, enorm_m, {'norma_m'});
        imprimir_resumo_vetor('Bloco 5 - Alinhamento estado sequenciador', ...
                              t_est, est_seq, {'estado'});
    end
end

% 6) Energia cinética
% -------------------------------------------------------------------------
% 
% if plot_sel == 0 || plot_sel == 6
% 
%     [t_e_base, e_base] = alinhar_tempo(t, energia_cinetica_base);
%     [t_e_elos, e_elos] = alinhar_tempo(t, energia_cinetica_elos);
%     [t_e_tot,  e_tot ] = alinhar_tempo(t, energia_cinetica_total);
% 
%     e_base = e_base(:,1);
%     e_elos = e_elos(:,1);
%     e_tot  = e_tot(:,1);
% 
%     N_e    = min([size(e_base,1), size(e_elos,1), size(e_tot,1)]);
%     t_e    = t_e_base(1:N_e);
%     e_base = e_base(1:N_e);
%     e_elos = e_elos(1:N_e);
%     e_tot  = e_tot(1:N_e);
% 
%     figure;
%     hold on;
%     h_base = plot(t_e, e_base, 'k', 'LineWidth', 1.5);
%     h_elos = plot(t_e, e_elos, 'r', 'LineWidth', 1.5);
%     h_tot  = plot(t_e, e_tot,  'b', 'LineWidth', 1.5);
%     h_sw = vline_mr(t_switch, lw_mr, mr_style);
%     h_ac = vline_acoplamento(t_acoplamento, lw_ac, ac_style);
%     hold off;
% 
%     xlabel('Tempo [s]');
%     ylabel('Energia cinética [J]');
%     grid on;
%     xlim([0 tempo_final]);
% 
%     legend([h_base, h_elos, h_tot, h_sw, h_ac], ...
%         {'Energia cinética do perseguidor [J]', ...
%          'Energia cinética dos elos [J]', ...
%          'Energia cinética total [J]', ...
%          'Início do modo operacional do MR', ...
%          'Instante de acoplamento'}, ...
%         'Location','eastoutside', 'Interpreter','none');
% 
% end


%% Função auxiliar: alinha tempo e sinal na primeira dimensão
function [t_use, y_use] = alinhar_tempo(t, y)
    t = t(:);
    [Ny, ~] = size(y);
    Nt      = numel(t);

    if Ny == 1 && Nt > 1
        y_use  = repmat(y, Nt, 1);
        t_use  = t;
    else
        Nmin   = min(Nt, Ny);
        t_use  = t(1:Nmin);
        y_use  = y(1:Nmin, :);
    end
end

%% Função auxiliar: escolhe o gráfico e plota
function ok = deve_imprimir(print_sel, idx_bloco)
    if ischar(print_sel)
        ok = strcmp(print_sel, 'all');
    else
        ok = (print_sel == 0) || (print_sel == idx_bloco);
    end
end

%% Função auxiliar: resumo numérico no console (robusta a NaN/Inf)
function imprimir_resumo_vetor(nome, t_local, Y, labels)
    if nargin < 4 || isempty(labels)
        labels = {};
    end

    t_local = t_local(:);
    if isempty(Y) || isempty(t_local)
        fprintf('\n[%s] sem dados.\n', nome);
        return;
    end

    % garante compatibilidade de tamanho (caso venha desalinhado)
    Nmin = min(numel(t_local), size(Y,1));
    t_local = t_local(1:Nmin);
    Y = Y(1:Nmin, :);

    % remove linhas onde o tempo não é finito ou onde TODAS as colunas são não-finitas
    mask_linha = isfinite(t_local) & any(isfinite(Y), 2);
    t_use = t_local(mask_linha);
    Y_use = Y(mask_linha, :);

    [N, ny] = size(Y_use);
    if N == 0
        fprintf('\n[%s] sem dados finitos.\n', nome);
        return;
    end

    dur = t_use(end) - t_use(1);

    % estimativa de passo e fs (opcional, mas útil)
    if N >= 2
        dt = diff(t_use);
        dt = dt(isfinite(dt) & dt > 0);
        if ~isempty(dt)
            dt_med = median(dt);
            fs_est = 1 / dt_med;
        else
            dt_med = NaN;
            fs_est = NaN;
        end
    else
        dt_med = NaN;
        fs_est = NaN;
    end

    fprintf('\n[%s]\n', nome);
    fprintf('  duração amostrada (finita): %.3f s, N = %d\n', dur, N);
    if ~isnan(dt_med)
        fprintf('  dt_med ≈ %.6g s, fs ≈ %.6g Hz\n', dt_med, fs_est);
    end

    for k = 1:ny
        yk = Y_use(:,k);
        mk = isfinite(yk);
        if ~any(mk)
            y_min = NaN; y_max = NaN; y_last = NaN;
            t_min = NaN; t_max = NaN;
            y_med = NaN; y_std = NaN; y_rms = NaN;
        else
            ykf = yk(mk);
            tkf = t_use(mk);

            [y_min, i_min] = min(ykf);
            [y_max, i_max] = max(ykf);
            t_min = tkf(i_min);
            t_max = tkf(i_max);

            y_last = ykf(end);
            y_med  = mean(ykf);
            y_std  = std(ykf);
            y_rms  = sqrt(mean(ykf.^2));
        end

        if ~isempty(labels) && numel(labels) >= k
            lab = labels{k};
        else
            lab = sprintf('coluna %d', k);
        end

        fprintf('  %s:\n', lab);
        fprintf('    min   = %.6g em t = %.6g s\n', y_min, t_min);
        fprintf('    max   = %.6g em t = %.6g s\n', y_max, t_max);
        fprintf('    média = %.6g, desvio = %.6g, RMS = %.6g\n', y_med, y_std, y_rms);
        fprintf('    final (último finito) = %.6g\n', y_last);
    end
end


%% Função única: linha vertical do início do modo operacional do MR
function h = vline_mr(t_switch, lw, style_str)
    if ~isnan(t_switch)
        h = xline(t_switch, style_str, 'LineWidth', lw);
    else
        h = plot(NaN, NaN, style_str, 'LineWidth', lw); % dummy para legenda
    end
end

%% Função única: linha vertical do instante de acoplamento
function h = vline_acoplamento(t_acoplamento, lw, style_str)
    if ~isnan(t_acoplamento)
        h = xline(t_acoplamento, style_str, 'LineWidth', lw);
    else
        h = plot(NaN, NaN, style_str, 'LineWidth', lw); % dummy para legenda
    end
end
function imprimir_resumo_erro_juntas(nome, t_local, E, labels, t_switch, t_acoplamento)
    t_local = t_local(:);
    if isempty(E) || isempty(t_local)
        fprintf('\n[%s] sem dados.\n', nome);
        return;
    end

    Nmin = min(numel(t_local), size(E,1));
    t_local = t_local(1:Nmin);
    E = E(1:Nmin,:);

    % só linhas finitas
    mask = isfinite(t_local) & any(isfinite(E),2);
    t_use = t_local(mask);
    E_use = E(mask,:);

    if isempty(t_use)
        fprintf('\n[%s] sem dados finitos.\n', nome);
        return;
    end

    fprintf('\n[%s]\n', nome);

    % segmentações
    segs = {};
    segs{end+1} = struct('nome','TOTAL', 'idx', true(size(t_use)));

    if ~isnan(t_switch)
        segs{end+1} = struct('nome','ANTES do MR', 'idx', t_use < t_switch);
        segs{end+1} = struct('nome','APÓS  MR',  'idx', t_use >= t_switch);
    end

    if ~isnan(t_acoplamento)
        segs{end+1} = struct('nome','ANTES do acoplamento', 'idx', t_use < t_acoplamento);
        segs{end+1} = struct('nome','APÓS  acoplamento',  'idx', t_use >= t_acoplamento);
    end

    for s = 1:numel(segs)
        idx = segs{s}.idx;
        if ~any(idx), continue; end

        ts = t_use(idx);
        Es = E_use(idx,:);

        dur = ts(end) - ts(1);
        fprintf('  [%s] duração = %.3f s, N = %d\n', segs{s}.nome, dur, numel(ts));

        for k = 1:size(Es,2)
            ek = Es(:,k);
            mk = isfinite(ek);
            if ~any(mk)
                continue;
            end
            ekf = ek(mk);
            tkf = ts(mk);

            [emin, imin] = min(ekf);
            [emax, imax] = max(ekf);
            eabsmax = max(abs(ekf));
            elast = ekf(end);

            emed = mean(ekf);
            estd = std(ekf);
            erms = sqrt(mean(ekf.^2));

            if ~isempty(labels) && numel(labels) >= k
                lab = labels{k};
            else
                lab = sprintf('j%d', k);
            end

            fprintf('    %s: min=%.6g (t=%.3f)  max=%.6g (t=%.3f)  |e|max=%.6g  RMS=%.6g  final=%.6g\n', ...
                lab, emin, tkf(imin), emax, tkf(imax), eabsmax, erms, elast);
        end
    end
end

function [t_sel, y_sel] = local_amostra(t_local, Y, t_ref)
    % mesma filosofia do seu imprimir_resumo_vetor: robusto a NaN/Inf e desalinhamento
    t_local = t_local(:);
    Nmin = min(numel(t_local), size(Y,1));
    t_local = t_local(1:Nmin);
    Y = Y(1:Nmin,:);

    mask = isfinite(t_local) & any(isfinite(Y),2);
    t_use = t_local(mask);
    Y_use = Y(mask,:);

    if isempty(t_use)
        t_sel = NaN;
        y_sel = [NaN NaN NaN];
        return;
    end

    [~, idx] = min(abs(t_use - t_ref));
    t_sel = t_use(idx);
    y_sel = Y_use(idx,:);
end