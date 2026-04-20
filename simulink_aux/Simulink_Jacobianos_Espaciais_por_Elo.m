function JacobianosEspaciais_porElo = Simulink_Jacobianos_Espaciais_por_Elo( ...
    matrizes_homogeneas, ...
    COM_elos_juntas_ferramenta_local, ...
    joint_axes)

%#codegen

% -------------------------------------------------------------------------
% Entradas:
%   matrizes_homogeneas                [4x4x5]
%   COM_elos_juntas_ferramenta_local   [3x3]  (COM local de cada elo)
%   joint_axes                         [3x3] (eixos das juntas em frame da junta)
%
% Saída:
%   JacobianosEspaciais_porElo         [6x1x3]
% -------------------------------------------------------------------------

numero_elos = 3;
numero_juntas = 3;

JacobianosEspaciais_porElo = zeros(6,1,numero_elos);

% Para cada elo i = 1..3
for indice_elo = 1:numero_elos

    % -------------------------------------------------------------
    % 1) Posição global do COM do elo i
    % -------------------------------------------------------------
    T_elo_i = matrizes_homogeneas(:,:,indice_elo);
    COM_local = COM_elos_juntas_ferramenta_local(:,indice_elo);
    COM_local_h = [COM_local; 1];

    COM_global_i = T_elo_i * COM_local_h;
    pos_COM_i = COM_global_i(1:3);

    % -------------------------------------------------------------
    % 2) Construção do Jacobiano espacial no COM
    % -------------------------------------------------------------
    Jacobiano_spacial_elo_i = zeros(6,1);

    % Varre cada junta j
    for indice_junta = 1:numero_juntas
        
        T_junta_j = matrizes_homogeneas(:,:,indice_junta);
        pos_junta_j = T_junta_j(1:3,4);

        R_junta_j = T_junta_j(1:3,1:3);
        eixo_junta_global = R_junta_j * joint_axes(:,indice_junta);

        vetor_posicao = pos_COM_i - pos_junta_j;

        velocidade_linear = cross(eixo_junta_global, vetor_posicao);
        velocidade_angular = eixo_junta_global;

        if indice_junta == indice_elo
            Jacobiano_spacial_elo_i(1:3) = velocidade_linear;
            Jacobiano_spacial_elo_i(4:6) = velocidade_angular;
        end
    end

    JacobianosEspaciais_porElo(:,:,indice_elo) = Jacobiano_spacial_elo_i;
end

end