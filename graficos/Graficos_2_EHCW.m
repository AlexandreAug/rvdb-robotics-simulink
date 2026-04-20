%% Graficos_2_EHCW.m

clear; clc; close all;

%% CARREGAR DADOS
% ======================================================================

load('trajetoria_Hill.mat');

% ----------------------------------------------------------------------
% Garantir formatos consistentes para LVLH
% ----------------------------------------------------------------------
t_LVLH = t_LVLH(:);              % vetor coluna

% Se pos_LVLH e vel_LVLH existirem, alinhar tamanhos
[N_pos,  ~] = size(pos_LVLH);
[N_vel,  ~] = size(vel_LVLH);
N_time      = numel(t_LVLH);

N = min([N_pos, N_vel, N_time]);

t_LVLH    = t_LVLH(1:N);
pos_LVLH  = pos_LVLH(1:N, :);
vel_LVLH  = vel_LVLH(1:N, :);

% Componentes LVLH
along  = pos_LVLH(:,1);
cross  = pos_LVLH(:,2);
radial = pos_LVLH(:,3);

v_along  = vel_LVLH(:,1);
v_cross  = vel_LVLH(:,2);
v_radial = vel_LVLH(:,3);

% Distância e norma da velocidade
dist_LVLH = vecnorm(pos_LVLH,2,2);      % [m]
vel_norm  = vecnorm(vel_LVLH,2,2);      % [m/s]

% ----------------------------------------------------------------------
% Velocidades nao saturadas (estruturas do Simulink: time + signals.values)
% ----------------------------------------------------------------------
t_vel_raw = veloc_radial_nao_sat.time(:);

vel_radial_raw     = veloc_radial_nao_sat.signals.values(:);
vel_along_raw      = veloc_alongtrack_nao_sat.signals.values(:);
vel_crosstrack_raw = veloc_crosstrack_nao_sat.signals.values(:);

N_vel_raw = min([ numel(t_vel_raw), ...
                  numel(vel_radial_raw), ...
                  numel(vel_along_raw), ...
                  numel(vel_crosstrack_raw), ...
                  numel(t_LVLH) ]);

t_vel_raw         = t_vel_raw(1:N_vel_raw);
vel_radial_raw    = vel_radial_raw(1:N_vel_raw);
vel_along_raw     = vel_along_raw(1:N_vel_raw);
vel_crosstrack_raw= vel_crosstrack_raw(1:N_vel_raw);


% Erros de posição (referência em 0 m)
erro_along  = -along;
erro_cross  = -cross;
erro_radial = -radial;

% Tempos de estabilização (último valor do vetor)
% ts_radial     = hill_pd_tempo_assentamento_radial(end);
% ts_alongtrack = hill_pd_tempo_assentamento_alongtrack(end);
% ts_crosstrack = hill_pd_tempo_assentamento_crosstrack(end);
ts_radial     = hill_pd_tempo_assentamento_radial_vec(end);
ts_alongtrack = hill_pd_tempo_assentamento_alongtrack_vec(end);
ts_crosstrack = hill_pd_tempo_assentamento_crosstrack_vec(end);


% ----------------------------------------------------------------------
% Garantir formatos consistentes para sinais de controle PD
% ----------------------------------------------------------------------
hill_pd_acel_radial          = hill_pd_acel_radial(:);
hill_pd_acel_alongtrack      = hill_pd_acel_alongtrack(:);
hill_pd_acel_crosstrack      = hill_pd_acel_crosstrack(:);

hill_pd_veloc_rel_max_radial     = hill_pd_veloc_rel_max_radial(:);
hill_pd_veloc_rel_max_alongtrack = hill_pd_veloc_rel_max_alongtrack(:);
hill_pd_veloc_rel_max_crosstrack = hill_pd_veloc_rel_max_crosstrack(:);

% alinhar comprimento dos sinais de aceleração com t_LVLH
N_PD_radial = min(numel(t_LVLH), numel(hill_pd_acel_radial));
N_PD_along  = min(numel(t_LVLH), numel(hill_pd_acel_alongtrack));
N_PD_cross  = min(numel(t_LVLH), numel(hill_pd_acel_crosstrack));

% usar o menor para definir t_PD comum (seguro)
N_PD = min([N_PD_radial, N_PD_along, N_PD_cross]);

t_PD = t_LVLH(1:N_PD);

hill_pd_acel_radial          = hill_pd_acel_radial(1:N_PD);
hill_pd_acel_alongtrack      = hill_pd_acel_alongtrack(1:N_PD);
hill_pd_acel_crosstrack      = hill_pd_acel_crosstrack(1:N_PD);

hill_pd_veloc_rel_max_radial     = hill_pd_veloc_rel_max_radial(1:N_PD);
hill_pd_veloc_rel_max_alongtrack = hill_pd_veloc_rel_max_alongtrack(1:N_PD);
hill_pd_veloc_rel_max_crosstrack = hill_pd_veloc_rel_max_crosstrack(1:N_PD);

%% FIGURA 1) Posição e distância relativa em LVLH
% ======================================================================

fig1 = figure('Name','Posicao e distancia relativa (LVLH)');

% 1a) Posição LVLH
hold on; grid on;

h_along  = plot(t_LVLH, along,  'LineWidth',3, 'Color','blue');   % along (azul)
h_cross  = plot(t_LVLH, cross,  'LineWidth',3, 'Color','red');    % cross (vermelho)
h_radial = plot(t_LVLH, radial, 'LineWidth',3, 'Color','black');  % radial (preto)

% Linhas de tempo de estabilização
h_ts_along  = xline(ts_alongtrack, '--', 'LineWidth',3, 'Color','blue');
h_ts_radial = xline(ts_radial,     '--', 'LineWidth',3, 'Color','black');

xlabel('Tempo [s]');
ylabel('Posicao relativa [m]');
% title('Posicao relativa em LVLH (LVLH)');
ylim([-400 50]);
xlim([0 900]);

hLeg1 = legend([h_along, h_cross, h_radial, h_ts_along, h_ts_radial], { ...
    'Along-track', ...
    'Cross-track', ...
    'Radial', ...
    'Tempo de estabilização along-track', ...
    'Tempo de estabilização radial'}, ...
    'Location','northeastoutside');

set(hLeg1,'FontSize',20);
set(gca,'FontSize',18);


%% FIGURA 2) Velocidades e norma da velocidade em LVLH
% ======================================================================

fig2 = figure('Name','Velocidades relativas (LVLH)');

% 2a) Velocidades LVLH
hold on; grid on;

% curvas de velocidade
h_valong  = plot(t_LVLH, v_along,  'LineWidth',3, 'Color','blue');     % azul
h_vcross  = plot(t_LVLH, v_cross,  'LineWidth',3, 'Color','red'); % vermelho
h_vradial = plot(t_LVLH, v_radial, 'LineWidth',3, 'Color','black'); % cinza escuro

% limites ±0,3 m/s (mais externos) — vermelho tracejado
h_lim03 = yline( 0.3, ':', 'LineWidth',2, 'Color','red');
yline(-0.3, ':', 'LineWidth',2, 'Color','red');

% limites ±0,03 m/s (faixa mais restrita)
h_lim003 = yline( 0.03, '-', 'LineWidth',1, 'Color','m');
yline(-0.03, '-', 'LineWidth',1, 'Color','m');

% linhas de tempo de estabilização (com handle para legenda)
h_ts_along  = xline(ts_alongtrack, '--', 'LineWidth',2, 'Color','blue');
h_ts_radial = xline(ts_radial,     '--', 'LineWidth',2, 'Color','black');

xlabel('Tempo [s]');
ylabel('Velocidade relativa [m/s]');
% title('Velocidades relativas em LVLH (LVLH)');
ylim([-0.1 0.35]);
xlim([0 900]);

hLeg2a = legend([h_valong, h_vcross, h_vradial, h_lim03, h_lim003, h_ts_along, h_ts_radial], { ...
    'Velocidade along-track', ...
    'Velocidade cross-track', ...
    'Velocidade radial', ...
    'Limite de velocidade \pm0,300 [m/s]', ...
    'Limite de velocidade \pm0,030 [m/s]', ...
    'Tempo de estabilização along-track', ...
    'Tempo de estabilização radial' ...
    }, ...
    'Location','northeastoutside');
set(hLeg2a,'FontSize',18);
set(gca,'FontSize',18);


%% FIGURA 3) Velocidades radial e along-track com limites
% ======================================================================

fig3 = figure('Name','Velocidades com limites de seguranca (LVLH)');
tiledlayout(fig3,2,1);

% 3a) Velocidade radial com limites fixos
nexttile;
hold on; grid on;

% velocidade radial simulada
h_radial = plot(t_LVLH, v_radial, 'LineWidth',3, 'Color','blue');

% limites ±0,3 m/s (mais externos) — vermelho tracejado
h_lim03 = yline( 0.3, ':', 'LineWidth',2, 'Color','red');
yline(-0.3, ':', 'LineWidth',2, 'Color','red');

% limites ±0,03 m/s (faixa mais restrita) — cinza/preto tracejado
h_lim003 = yline( 0.03, ':', 'LineWidth',2, 'Color','black');
yline(-0.03, ':', 'LineWidth',2, 'Color','black');

% Linhas de tempo de estabilização (handles para a legenda)
h_ts_along  = xline(ts_alongtrack, '--', 'LineWidth',3, 'Color','blue');
h_ts_radial = xline(ts_radial,     '--', 'LineWidth',3, 'Color','black');

xlabel('Tempo [s]');
ylabel('Velocidade radial [m/s]');
% title('Velocidade radial e limites de seguranca (LVLH)');
xlim([0 900]);
ylim([-0.5 0.5]);

hLeg3a = legend([h_radial, h_lim03, h_lim003, h_ts_along, h_ts_radial], { ...
    'Velocidade radial', ...
    'Limite de velocidade \pm0,300 [m/s]', ...
    'Limite de velocidade \pm0,030 [m/s]', ...
    'Tempo de estabilização along-track', ...
    'Tempo de estabilização radial' ...
    }, ...
    'Location','northeastoutside');
set(hLeg3a,'FontSize',18);
set(gca,'FontSize',18);

% 3b) Velocidade along-track com limites fixos
nexttile;
hold on; grid on;

% velocidade along-track simulada
h_along = plot(t_LVLH, v_along, 'LineWidth',3, 'Color','blue');

% limites ±0,3 m/s (mais externos) — vermelho tracejado
h_lim03_along = yline( 0.3, ':', 'LineWidth',2, 'Color','red');
yline(-0.3, ':', 'LineWidth',2, 'Color','red');

% limites ±0,03 m/s (faixa mais restrita) — cinza tracejado
h_lim003_along = yline( 0.03, ':', 'LineWidth',2, 'Color','black');
yline(-0.03, ':', 'LineWidth',2, 'Color','black');

% Linhas de tempo de estabilização (handles para a legenda)
h_ts_along2  = xline(ts_alongtrack, '--', 'LineWidth',3, 'Color','blue');
h_ts_radial2 = xline(ts_radial,     '--', 'LineWidth',3, 'Color','black');

xlabel('Tempo [s]');
ylabel('Velocidade along-track [m/s]');
% title('Velocidade along-track e limites de seguranca (LVLH)');
xlim([0 900]);
ylim([-0.5 0.5]);

hLeg3b = legend([h_along, h_lim03_along, h_lim003_along, h_ts_along2, h_ts_radial2], { ...
    'Velocidade along-track', ...
    'Limite de velocidade \pm0,300 [m/s]', ...
    'Limite de velocidade \pm0,030 [m/s]', ...
    'Tempo de estabilização along-track', ...
    'Tempo de estabilização radial' ...
    }, ...
    'Location','northeastoutside');
set(hLeg3b,'FontSize',18);
set(gca,'FontSize',18);


%% =====================================================================
% FIGURA 4) Trajetoria 3D em LVLH
% ======================================================================

fig4 = figure('Name','Trajetoria relativa em LVLH (3D)');
hold on; grid on; axis equal;

h_traj3D = plot3(along, cross, radial, 'LineWidth',3, 'Color','blue');
h_ini3D  = plot3(along(1), cross(1), radial(1), 'o', ...
    'MarkerSize',15, 'LineWidth',2, 'Color',[0 0 0], 'MarkerFaceColor',[0 0 0]);
h_fim3D  = plot3(along(end), cross(end), radial(end), 'x', ...
    'MarkerSize',30, 'LineWidth',2, 'Color','red');

xlabel('Along-track [m]');
ylabel('Cross-track [m]');
zlabel('Radial [m]');
% title('Trajetoria relativa em LVLH (LVLH)');

hLeg4 = legend([h_traj3D, h_ini3D, h_fim3D], ...
       {'Trajetoria do perseguidor','Posicao inicial','Posicao final'}, ...
       'Location','northeastoutside');

set(gca,'FontSize',18);      % eixos, ticks, labels e título
set(hLeg4,'FontSize',18);    % legenda

view(35,20);

%% =====================================================================
% FIGURA 5) Projecoes 2D da trajetoria em LVLH
% ======================================================================

fig5 = figure('Name','Projecoes 2D da trajetoria em LVLH');
tiledlayout(fig5,3,1);   % tres subplots do mesmo tamanho (verticais)

% 5a) Projecao along x radial
nexttile;
hold on; grid on;
plot(along, radial, 'LineWidth',3, 'Color','blue');
xlabel('Along-track [m]');
ylabel('Radial [m]');
xlim([-370 10]);
ylim([-110 10]);
hLeg5a = legend({'Trajetoria do perseguidor'}, 'Location','northeastoutside');
set(gca,'FontSize',18);
set(hLeg5a,'FontSize',18);

% 5b) Projecao along x cross
nexttile;
hold on; grid on;
plot(along, cross, 'LineWidth',3, 'Color','blue');
xlabel('Along-track [m]');
ylabel('Cross-track [m]');
xlim([-370 10]);
ylim([-10 10]);
hLeg5b = legend({'Trajetoria do perseguidor'}, 'Location','northeastoutside');
set(gca,'FontSize',18);
set(hLeg5b,'FontSize',18);

% 5c) Projecao radial x cross
nexttile;
hold on; grid on;
plot(radial, cross, 'LineWidth',3, 'Color','blue');
xlabel('Radial [m]');
ylabel('Cross-track [m]');
xlim([-110 10]);
ylim([-10 10]);
hLeg5c = legend({'Trajetoria do perseguidor'}, 'Location','northeastoutside');
set(gca,'FontSize',18);
set(hLeg5c,'FontSize',18);

%% FIGURA 6a) Erro de posicao radial e tempo de estabilização
% ======================================================================

fig5a = figure('Name','Erro de posicao radial (LVLH)');
% ---------- eixo principal ----------
axMain = axes('Parent',fig5a);
hold(axMain,'on'); grid(axMain,'on');

% Faixa de tolerancia (ajuste conforme criterio)
tol_pos = 0.5;  % exemplo: 0,5 m

% curva principal
h_err = plot(axMain, t_LVLH, erro_radial, 'LineWidth',2.0, 'Color','blue');

% Faixa de tolerancia centrada no valor final
h_tol = yline(axMain, erro_radial(end)+tol_pos, 'LineStyle','--', 'Color','black');
yline(axMain, erro_radial(end)-tol_pos,         'LineStyle','--', 'Color','black');

% linha de tempo de estabilização
h_ts = xline(axMain, ts_radial, 'LineStyle','--', 'Color','red', 'LineWidth',2);

xlabel(axMain,'Tempo [s]');  xlim(axMain,[0 600]);
ylabel(axMain,'Erro radial [m]'); ylim(axMain,[-10 110]);

hLeg = legend(axMain, ...
    [h_err, h_tol, h_ts], ...
    {'Erro radial','Faixa de tolerancia','Tempo de estabilização'}, ...
    'Location','northeastoutside');
set(hLeg, 'FontSize', 14);
set(axMain, 'FontSize', 14);


% ---------- inset de zoom ----------
% posição do zoom na figura [x y largura altura] (coordenadas normalizadas)
insetPos = [0.30 0.60 0.30 0.30];
axInset  = axes('Parent',fig5a, 'Position', insetPos);
hold(axInset,'on'); grid(axInset,'on');

% mesma curva e mesmas linhas no zoom
plot(axInset, t_LVLH, erro_radial, 'LineWidth',2, 'Color','blue');
yline(axInset, erro_radial(end)+0.5, 'LineWidth',3,'LineStyle','--', 'Color','black');
yline(axInset, erro_radial(end)-0.5, 'LineWidth',3,'LineStyle','--', 'Color','black');
xline(axInset, ts_radial, 'LineStyle','--', 'Color','red', 'LineWidth',1.5);

% janela de zoom em torno do estabilização
xlim(axInset,[450 600]);   % ajuste fino se quiser
y_center = erro_radial(end);
ylim(axInset,[y_center-1  y_center+1]);

set(axInset,'FontSize',14);
box(axInset,'on');

%% FIGURA 6b) Erro de posicao along-track e tempo de estabilização
% ======================================================================

fig5b = figure('Name','Erro de posicao along-track (LVLH)');

% ---------- eixo principal ----------
axMain_b = axes('Parent',fig5b);
hold(axMain_b,'on'); grid(axMain_b,'on');

% curva principal
h_err_b = plot(t_LVLH, erro_along, 'LineWidth',2.0, 'Color','blue');

% Faixa de tolerancia centrada no valor final
h_tol_sup_b = yline(erro_along(end)+0.05, 'LineStyle','--', 'Color','black');
yline(erro_along(end)-0.05, 'LineStyle','--', 'Color','black');

% linha de tempo de estabilização
h_ts_b = xline(ts_alongtrack, 'LineStyle',':', 'Color','red', 'LineWidth',2);

% linha de tempo de estabilização
xline(axMain_b, ts_alongtrack, 'LineStyle',':', 'Color','red', 'LineWidth',2);

xlabel(axMain_b,'Tempo [s]');  xlim(axMain_b,[0 600]);
ylabel(axMain_b,'Erro along-track [m]'); ylim(axMain_b,[-10 370]);
set(axMain_b,'FontSize',18);  % aumenta fonte dos eixos e ticks

hLeg_b = legend([h_err_b, h_tol_sup_b, h_ts_b], ...
    {'Erro along-track','Faixa de tolerancia','Tempo de estabilização'}, ...
    'Location','northeastoutside');
set(hLeg_b,'FontSize',18);

% ---------- inset de zoom ----------
insetPos_b = [0.30 0.60 0.30 0.30];   % [x y largura altura] em coordenadas normalizadas
axInset_b  = axes('Parent',fig5b, 'Position', insetPos_b);
hold(axInset_b,'on'); grid(axInset_b,'on');

% mesma curva e mesmas linhas no zoom
plot(axInset_b, t_LVLH, erro_along, 'LineWidth',2, 'Color','blue');
yline(axInset_b, erro_along(end)+0.5, 'LineWidth',3,'LineStyle','--', 'Color','black');
yline(axInset_b, erro_along(end)-0.5, 'LineWidth',3,'LineStyle','--', 'Color','black');
xline(axInset_b, ts_alongtrack, 'LineStyle',':', 'Color','red', 'LineWidth',1.5);

% janela de zoom em torno do estabilização
xlim(axInset_b,[450 600]);             % ajuste fino se quiser
y_center_b = erro_along(end);
ylim(axInset_b,[y_center_b-1  y_center_b+1]);

set(axInset_b,'FontSize',10);
box(axInset_b,'on');

%% FIGURA 6c) Erro de posicao cross-track e tempo de estabilização
% ======================================================================
fig5c = figure('Name','Erro de posicao cross-track (LVLH)');
hold on; grid on;

h_cross = plot(t_LVLH, erro_cross, 'LineWidth',2.0, 'Color','blue');
h_tol1  = yline(erro_cross(end)+0.05, 'LineWidth',3,'LineStyle','--', 'Color','black');
yline(erro_cross(end)-0.05, 'LineWidth',3,'LineStyle','--', 'Color','black');  % mesma cor, sem ir pra legenda
h_ts    = xline(ts_crosstrack, 'LineStyle',':', 'Color','red', 'LineWidth',2);

xlabel('Tempo [s]');
ylabel('Erro cross-track [m]');
xlim([-10 600]);
ylim([-0.1 0.1]);

set(gca,'FontSize',18);

hLeg = legend([h_cross, h_tol1, h_ts], ...
    {'Erro cross-track','Faixa de tolerancia','Tempo de estabilização'}, ...
    'Location','northeastoutside');
set(hLeg,'FontSize',18);

%% FIGURA 7) Aceleracoes de controle PD
% ======================================================================

fig6 = figure('Name','Acelerações de controle PD (LVLH)');
tiledlayout(fig6,3,1);

% 7a) Aceleração radial
nexttile;
hold on; grid on;

h_radial  = plot(t_PD, hill_pd_acel_radial, 'LineWidth',2.0, 'Color','black');
h_ts_rad  = xline(ts_radial, '--', 'Color','black', 'LineWidth',2);

xlabel('Tempo [s]');
ylabel('Aceleração radial [m/s^2]');
xlim([0 900]);
ylim([-1.1 1.1]);
% title('Aceleração de controle radial (LVLH)');

hLeg7a = legend([h_radial, h_ts_rad], ...
    {'Acel. radial','Tempo de estabilização radial'}, ...
    'Location','northeastoutside');
set(hLeg7a,'FontSize',18);
set(gca,'FontSize',18);

% 7b) Aceleração along-track
nexttile;
hold on; grid on;

h_along  = plot(t_PD, hill_pd_acel_alongtrack, 'LineWidth',2.0, 'Color','blue');
h_ts_al  = xline(ts_alongtrack, '--', 'Color','blue', 'LineWidth',2);

xlabel('Tempo [s]');
ylabel('Acel. along-track');
xlim([0 900]);
ylim([-1.1 1.1]);
% title('Aceleração de controle along-track (LVLH)');

hLeg7b = legend([h_along, h_ts_al], ...
    {'Aceleração along-track','Tempo de estabilização along-track'}, ...
    'Location','northeastoutside');
set(hLeg7b,'FontSize',18);
set(gca,'FontSize',18);

% 7c) Aceleração cross-track
nexttile;
hold on; grid on;

h_cross = plot(t_PD, hill_pd_acel_crosstrack, 'LineWidth',2.0, 'Color','red');
h_ts_cr = xline(ts_crosstrack, '--', 'Color','red', 'LineWidth',2);

xlabel('Tempo [s]');
ylabel('Acel. cross-track');
xlim([0 900]);
ylim([-1.1 1.1]);
% title('Aceleração de controle cross-track (LVLH)');

hLeg7c = legend([h_cross, h_ts_cr], ...
    {'Aceleração cross-track','Tempo de estabilização cross-track'}, ...
    'Location','northeastoutside');
set(hLeg7c,'FontSize',18);
set(gca,'FontSize',18);


%% FIGURA 8) Velocidade antes e depois da saturacao (LVLH)
% ======================================================================

fig7 = figure('Name','Velocidade antes e depois da saturacao (LVLH)');
tiledlayout(fig7,3,1);

% 8a) Radial
nexttile;
hold on; grid on;
plot(t_vel_raw, vel_radial_raw, 'LineWidth',1.5, 'Color','red');
plot(t_LVLH,    v_radial,       'LineWidth',2.0, 'Color','black');
xlabel('Tempo [s]');
ylabel('Velocidade radial [m/s]');
xlim([0 900]);
ylim([-0.5 5]);
% title('Velocidade radial antes e depois da saturacao (LVLH)');
hLeg8a = legend({'Velocidade radial nao saturada', ...
                 'Velocidade radial saturada'}, ...
                 'Location','northeastoutside');
set(hLeg8a,'FontSize',18);
set(gca,'FontSize',18);

% 8b) Along-track
nexttile;
hold on; grid on;
plot(t_vel_raw, vel_along_raw, 'LineWidth',1.5, 'Color','red');
plot(t_LVLH,    v_along,       'LineWidth',2.0, 'Color','blue');
xlabel('Tempo [s]');
ylabel('Veloc. along-track');
xlim([0 900]);
ylim([-0.5 10]);
% title('Velocidade along-track antes e depois da saturacao (LVLH)');
hLeg8b = legend({'Velocidade along-track nao saturada', ...
                 'Velocidade along-track saturada'}, ...
                 'Location','northeastoutside');
set(hLeg8b,'FontSize',18);
set(gca,'FontSize',18);

% 8c) Cross-track
nexttile;
hold on; grid on;
plot(t_vel_raw, vel_crosstrack_raw, 'LineWidth',2,   'Color','red');
plot(t_LVLH,    v_cross,            'LineWidth',2.0, 'Color','black');
xlabel('Tempo [s]');
ylabel('Veloc. cross-track');
xlim([0 900]);
ylim([-0.5 0.5]);
% title('Velocidade cross-track antes e depois da saturacao (LVLH)');
hLeg8c = legend({'Velocidade cross-track nao saturada', ...
                 'Velocidade cross-track saturada'}, ...
                 'Location','northeastoutside');
set(hLeg8c,'FontSize',18);
set(gca,'FontSize',18);

%% FIGURA 9) Distância relativa
% Mostra onde inicia cada etapa da simulação

%% Distância relativa LVLH (apenas 1 gráfico)
fig1 = figure('Name','Posicao e distancia relativa (LVLH)');
hold on; grid on;

% curva principal: distância relativa (linha vermelha original)
h_dist = plot(t_LVLH, dist_LVLH, ...
    'LineWidth', 3, ...
    'Color', 'red');

% limites de eixo antes de desenhar os marcadores verticais
ylim([-50 400]);
xlim([0 900]);

% 1) Bolinha preta no instante inicial
t0 = t_LVLH(1);
h_inicio_longa = plot(t0, dist_LVLH(1), ...
    'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'k');

% 2) Linha vertical "pontilhada" com bolinhas em t = 67,57 s (verde)
t_app_final = 67.57;
yl = ylim;                                % usa os limites já definidos
y_line = linspace(yl(1), yl(2), 30);      % pontos ao longo do eixo y
x_line = t_app_final * ones(size(y_line));

h_app_final = plot(x_line, y_line, ...
    'o-', ...
    'LineWidth', 1.5, ...
    'MarkerSize', 4, ...
    'Color', [0 0.7 0], ...               % verde não muito escuro
    'MarkerFaceColor', [0 0.7 0]);

% 3) Linhas verticais de tempo de estabilização (mesmas cores do subplot 1)
h_ts_along  = xline(ts_alongtrack, '--', ...
    'LineWidth', 3, ...
    'Color', 'blue');
h_ts_radial = xline(ts_radial, '--', ...
    'LineWidth', 3, ...
    'Color', 'black');

% Rótulos de eixos
xlabel('Tempo [s]');
ylabel('Distancia relativa [m]');

% Legenda
hLeg = legend([h_dist, h_inicio_longa, h_app_final, h_ts_along, h_ts_radial], { ...
    'Distancia relativa', ...
    'Início da aproximação de longa distância', ...
    'Início da aproximação final', ...
    'Tempo de estabilização along-track', ...
    'Tempo de estabilização radial'}, ...
    'Location', 'northeastoutside');

set(hLeg, 'FontSize', 14);
set(gca,  'FontSize', 14);


%% RESUMO NUMERICO NO COMMAND WINDOW
% ======================================================================

% Função auxiliar para tempo em hh:mm:ss
fmtHMS = @(t) sprintf('%02dh:%02dm:%02ds', ...
    floor(t/3600), floor(mod(t,3600)/60), floor(mod(t,60)));

% Tempos inicial e final em LVLH
t0 = t_LVLH(1);
tf = t_LVLH(end);

% Posições e velocidades inicial e final em LVLH
pos0 = pos_LVLH(1,:);      % [along, cross, radial] inicial [m]
posf = pos_LVLH(end,:);    % final

vel0 = vel_LVLH(1,:);      % [v_along, v_cross, v_radial] inicial [m/s]
velf = vel_LVLH(end,:);    % final

% Normas
norm_pos0 = norm(pos0);
norm_posf = norm(posf);

norm_vel0 = norm(vel0);
norm_velf = norm(velf);

% Picos absolutos (posicao e velocidade por componente)
max_abs_pos = max(abs(pos_LVLH),[],1);   % [along, cross, radial]
max_abs_vel = max(abs(vel_LVLH),[],1);   % [v_along, v_cross, v_radial]

% Picos de norma
max_dist      = max(dist_LVLH);
max_vel_norm  = max(vel_norm);

% Picos de aceleracao de controle
max_abs_acel_radial     = max(abs(hill_pd_acel_radial));
max_abs_acel_alongtrack = max(abs(hill_pd_acel_alongtrack));
max_abs_acel_crosstrack = max(abs(hill_pd_acel_crosstrack));

% Picos de limites de velocidade
max_vmax_radial     = max(hill_pd_veloc_rel_max_radial);
max_vmax_alongtrack = max(hill_pd_veloc_rel_max_alongtrack);
max_vmax_crosstrack = max(hill_pd_veloc_rel_max_crosstrack);

fprintf('\n================ RESUMO MOVIMENTO RELATIVO HILL/CW ================\n');

% fprintf('Tempo inicial LVLH           : %8.3f s (%s)\n', t0, fmtHMS(t0));
% fprintf('Tempo final LVLH             : %8.3f s (%s)\n', tf, fmtHMS(tf));
% fprintf('Duracao da simulacao         : %8.3f s (%s)\n\n', tf - t0, fmtHMS(tf - t0));
% 
% fprintf('Posicao relativa inicial LVLH [m]:\n');
% fprintf('  Along-track                : %+10.3f\n', pos0(1));
% fprintf('  Cross-track                : %+10.3f\n', pos0(2));
% fprintf('  Radial                     : %+10.3f\n', pos0(3));
% 
% fprintf('\nPosicao relativa final LVLH [m]:\n');
% fprintf('  Along-track                : %+10.3f\n', posf(1));
% fprintf('  Cross-track                : %+10.3f\n', posf(2));
% fprintf('  Radial                     : %+10.3f\n', posf(3));
% 
% fprintf('\nNorma da posicao relativa LVLH:\n');
% fprintf('  Inicial                    : %10.3f m (%8.6f km)\n', norm_pos0, norm_pos0/1000);
% fprintf('  Final                      : %10.3f m (%8.6f km)\n', norm_posf, norm_posf/1000);
% fprintf('  Maxima                     : %10.3f m (%8.6f km)\n\n', max_dist, max_dist/1000);
% 
% fprintf('Velocidade relativa inicial LVLH [m/s]:\n');
% fprintf('  v along-track              : %+10.6f\n', vel0(1));
% fprintf('  v cross-track              : %+10.6f\n', vel0(2));
% fprintf('  v radial                   : %+10.6f\n', vel0(3));
% 
% fprintf('\nVelocidade relativa final LVLH [m/s]:\n');
% fprintf('  v along-track              : %+10.6f\n', velf(1));
% fprintf('  v cross-track              : %+10.6f\n', velf(2));
% fprintf('  v radial                   : %+10.6f\n', velf(3));
% 
% fprintf('\nNorma da velocidade relativa LVLH:\n');
% fprintf('  Inicial                    : %10.6f m/s\n', norm_vel0);
% fprintf('  Final                      : %10.6f m/s\n', norm_velf);
% fprintf('  Maxima                     : %10.6f m/s\n\n', max_vel_norm);
% 
% fprintf('Picos absolutos de posicao LVLH [m]:\n');
% fprintf('  max |along-track|          : %10.3f m\n', max_abs_pos(1));
% fprintf('  max |cross-track|          : %10.3f m\n', max_abs_pos(2));
% fprintf('  max |radial|               : %10.3f m\n\n', max_abs_pos(3));
% 
% fprintf('Picos absolutos de velocidade LVLH [m/s]:\n');
% fprintf('  max |v along-track|        : %10.6f m/s\n', max_abs_vel(1));
% fprintf('  max |v cross-track|        : %10.6f m/s\n', max_abs_vel(2));
% fprintf('  max |v radial|             : %10.6f m/s\n\n', max_abs_vel(3));

fprintf('Tempos de estabilização (criterio adotado no bloco de medicao):\n');
fprintf('  Radial                     : %10.3f s (%s)\n', ts_radial,     fmtHMS(ts_radial));
fprintf('  Along-track                : %10.3f s (%s)\n', ts_alongtrack, fmtHMS(ts_alongtrack));
fprintf('  Cross-track                : %10.3f s (%s)\n\n', ts_crosstrack, fmtHMS(ts_crosstrack));

% fprintf('Picos de aceleracao de controle PD [m/s^2]:\n');
% fprintf('  max |acel radial|          : %10.6f\n', max_abs_acel_radial);
% fprintf('  max |acel along-track|     : %10.6f\n', max_abs_acel_alongtrack);
% fprintf('  max |acel cross-track|     : %10.6f\n\n', max_abs_acel_crosstrack);
% 
% fprintf('Picos dos limites de velocidade relativa [m/s]:\n');
% fprintf('  max limite radial          : %10.6f\n', max_vmax_radial);
% fprintf('  max limite along-track     : %10.6f\n', max_vmax_alongtrack);
% fprintf('  max limite cross-track     : %10.6f\n', max_vmax_crosstrack);

% % Picos de velocidades nao saturadas
% max_abs_vel_radial_raw     = max(abs(vel_radial_raw));
% max_abs_vel_alongtrack_raw = max(abs(vel_along_raw));
% max_abs_vel_crosstrack_raw = max(abs(vel_crosstrack_raw));
% 
% fprintf('\nPicos de velocidade relativa NAO saturada [m/s]:\n');
% fprintf('  max |v radial nao sat|     : %10.6f m/s\n', max_abs_vel_radial_raw);
% fprintf('  max |v along nao sat|      : %10.6f m/s\n', max_abs_vel_alongtrack_raw);
% fprintf('  max |v cross nao sat|      : %10.6f m/s\n', max_abs_vel_crosstrack_raw);

fprintf('\n=================================================================\n\n');
