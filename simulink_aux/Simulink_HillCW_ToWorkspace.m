%% Simulink_HillCW_ToWorkspace.m

MR_Setup_Simulacao;
TransferenciaOrbital;

if ~exist('TransfOrbital','var') || isempty(TransfOrbital)
    TransfOrbital = TransferenciaOrbital();  % chama sua função e pega o struct
end

TO = TransfOrbital;  % atalho

out = sim('Simulink_HillCW.slx');

%% Saídas vindas do Simulink
pos_radial   = out.hill_radial;
pos_along    = out.hill_alongtrack;
pos_cross    = out.hill_crosstrack;
    veloc_radial = out.hill_veloc_radial;     % veloc com saturação
    veloc_along  = out.hill_veloc_alongtrack; % veloc com saturação
    veloc_cross  = out.hill_veloc_crosstrack; % veloc com saturação
        veloc_radial_nao_sat     = out.hill_veloc_radial_raw;
        veloc_alongtrack_nao_sat = out.hill_veloc_alongtrack_raw;
        veloc_crosstrack_nao_sat = out.hill_veloc_crosstrack_raw;
pos_ECI_x    = out.hill_ECI_pos_x;
pos_ECI_y    = out.hill_ECI_pos_y;
pos_ECI_z    = out.hill_ECI_pos_z;
    veloc_ECI_x  = out.hill_ECI_veloc_x;
    veloc_ECI_y  = out.hill_ECI_veloc_y;
    veloc_ECI_z  = out.hill_ECI_veloc_z;

%% 1) Tempo LVLH
t_LVLH = pos_radial.time;   % [N1 x 1]

% 2) Matriz LVLH (along, cross, radial)
pos_LVLH = [ ...
    pos_along.signals.values, ...
    pos_cross.signals.values, ...
    pos_radial.signals.values ];

vel_LVLH = [ ...
    veloc_along.signals.values, ...
    veloc_cross.signals.values, ...
    veloc_radial.signals.values ];

% 3) Tempo ECI
t_ECI = pos_ECI_x.time;     

% 4) Matriz ECI (x, y, z)
pos_ECI = [ ...
    pos_ECI_x.signals.values, ...
    pos_ECI_y.signals.values, ...
    pos_ECI_z.signals.values ];

veloc_ECI = [ ...
    veloc_ECI_x.signals.values, ...
    veloc_ECI_y.signals.values, ...
    veloc_ECI_z.signals.values ];

%% Bloco "controlador hill pd radial"
hill_pd_acel_radial          = out.hill_pd_acel_radial.signals.values;
hill_pd_veloc_rel_max_radial = out.hill_pd_veloc_rel_max_radial.signals.values;

%% Bloco "controlador hill pd alongtrack"
hill_pd_acel_alongtrack          = out.hill_pd_acel_alongtrack.signals.values;
hill_pd_veloc_rel_max_alongtrack = out.hill_pd_veloc_rel_max_alongtrack.signals.values;

%% Bloco "controlador hill pd crosstrack"
hill_pd_acel_crosstrack          = out.hill_pd_acel_crosstrack.signals.values;
hill_pd_veloc_rel_max_crosstrack = out.hill_pd_veloc_rel_max_crosstrack.signals.values;

%% Bloco "medir tempo assentamento"
hill_pd_tempo_assentamento_radial     = out.hill_pd_tempo_assentamento_radial.signals.values;
hill_pd_tempo_assentamento_alongtrack = out.hill_pd_tempo_assentamento_alongtrack.signals.values;
hill_pd_tempo_assentamento_crosstrack = out.hill_pd_tempo_assentamento_crosstrack.signals.values;

% Garantir que sejam vetores coluna
hill_pd_tempo_assentamento_radial_vec     = hill_pd_tempo_assentamento_radial(:);
hill_pd_tempo_assentamento_alongtrack_vec = hill_pd_tempo_assentamento_alongtrack(:);
hill_pd_tempo_assentamento_crosstrack_vec = hill_pd_tempo_assentamento_crosstrack(:);

% escalares (usa o último valor do vetor)
ts_radial     = hill_pd_tempo_assentamento_radial_vec(end);
ts_alongtrack = hill_pd_tempo_assentamento_alongtrack_vec(end);
ts_crosstrack = hill_pd_tempo_assentamento_crosstrack_vec(end);

%% Ajuste de formatos e comprimentos

% tempos como vetores coluna
t_LVLH = t_LVLH(:);
t_ECI  = t_ECI(:);

% --- LVLH: alinhar pelo menor comprimento ---
N_LVLH = min([numel(t_LVLH), size(pos_LVLH,1), size(vel_LVLH,1)]); 

t_LVLH   = t_LVLH(1:N_LVLH);
pos_LVLH = pos_LVLH(1:N_LVLH, :);
vel_LVLH = vel_LVLH(1:N_LVLH, :);

% --- ECI: reshape 1x3xN -> N x 3 ---
pos_ECI   = squeeze(pos_ECI);    % 3 x N
veloc_ECI = squeeze(veloc_ECI);  % 3 x N

pos_ECI   = pos_ECI.';           % N x 3
veloc_ECI = veloc_ECI.';         % N x 3

N_ECI = min([numel(t_ECI), size(pos_ECI,1), size(veloc_ECI,1)]);   

t_ECI     = t_ECI(1:N_ECI);
pos_ECI   = pos_ECI(1:N_ECI, :);
veloc_ECI = veloc_ECI(1:N_ECI, :);

% 5) Salva em .mat
save('trajetoria_Hill.mat', ...
    't_LVLH','pos_LVLH','vel_LVLH', ...
    't_ECI','pos_ECI','veloc_ECI', ...
    'hill_pd_acel_radial', ...
    'hill_pd_veloc_rel_max_radial', ...
    'hill_pd_acel_alongtrack', ...
    'hill_pd_veloc_rel_max_alongtrack', ...
    'hill_pd_acel_crosstrack', ...
    'hill_pd_veloc_rel_max_crosstrack', ...
    'hill_pd_tempo_assentamento_radial_vec', ...
    'hill_pd_tempo_assentamento_alongtrack_vec', ...
    'hill_pd_tempo_assentamento_crosstrack_vec', ...
    'ts_radial', 'ts_alongtrack', 'ts_crosstrack', ...
    'veloc_radial_nao_sat', ...
    'veloc_alongtrack_nao_sat', ...
    'veloc_crosstrack_nao_sat');

%% Checagem de tamanhos das principais variáveis

fprintf('\n=== LVLH ===\n');
fprintf('size(t_LVLH)                       = '); disp(size(t_LVLH));
fprintf('size(pos_LVLH)                     = '); disp(size(pos_LVLH));
fprintf('size(vel_LVLH)                     = '); disp(size(vel_LVLH));

fprintf('\n=== ECI ===\n');
fprintf('size(t_ECI)                        = '); disp(size(t_ECI));
fprintf('size(pos_ECI)                      = '); disp(size(pos_ECI));
fprintf('size(veloc_ECI)                    = '); disp(size(veloc_ECI));

fprintf('\n=== Controlador Hill PD (acelerações) ===\n');
fprintf('size(hill_pd_acel_radial)          = '); disp(size(hill_pd_acel_radial));
fprintf('size(hill_pd_acel_alongtrack)      = '); disp(size(hill_pd_acel_alongtrack));
fprintf('size(hill_pd_acel_crosstrack)      = '); disp(size(hill_pd_acel_crosstrack));

fprintf('\n=== Controlador Hill PD (veloc. rel. máx.) ===\n');
fprintf('size(hill_pd_veloc_rel_max_radial)     = '); disp(size(hill_pd_veloc_rel_max_radial));
fprintf('size(hill_pd_veloc_rel_max_alongtrack) = '); disp(size(hill_pd_veloc_rel_max_alongtrack));
fprintf('size(hill_pd_veloc_rel_max_crosstrack) = '); disp(size(hill_pd_veloc_rel_max_crosstrack));

fprintf('\n=== Tempos de assentamento (vetores) ===\n');
fprintf('size(hill_pd_tempo_assentamento_radial_vec)     = '); disp(size(hill_pd_tempo_assentamento_radial_vec));
fprintf('size(hill_pd_tempo_assentamento_alongtrack_vec) = '); disp(size(hill_pd_tempo_assentamento_alongtrack_vec));
fprintf('size(hill_pd_tempo_assentamento_crosstrack_vec) = '); disp(size(hill_pd_tempo_assentamento_crosstrack_vec));

fprintf('\n=== Velocidades não saturadas LVLH ===\n');
fprintf('size(veloc_radial_nao_sat)         = '); disp(size(veloc_radial_nao_sat));
fprintf('size(veloc_alongtrack_nao_sat)     = '); disp(size(veloc_alongtrack_nao_sat));
fprintf('size(veloc_crosstrack_nao_sat)     = '); disp(size(veloc_crosstrack_nao_sat));

fprintf('\n=== Conferência dos inputs salvos ===\n');
% se já tiver montado em outro script:
% pos_input_LVLH, veloc_input_LVLH, pos_input_ECI, veloc_input_ECI
if exist('pos_input_LVLH','var')
    fprintf('size(pos_input_LVLH)               = '); disp(size(pos_input_LVLH));
end
if exist('veloc_input_LVLH','var')
    fprintf('size(veloc_input_LVLH)             = '); disp(size(veloc_input_LVLH));
end
if exist('pos_input_ECI','var')
    fprintf('size(pos_input_ECI)                = '); disp(size(pos_input_ECI));
end
if exist('veloc_input_ECI','var')
    fprintf('size(veloc_input_ECI)              = '); disp(size(veloc_input_ECI));
end
