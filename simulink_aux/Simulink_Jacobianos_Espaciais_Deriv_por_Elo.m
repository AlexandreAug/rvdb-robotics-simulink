function JacobianosEspaciais_deriv_porElo = Simulink_Jacobianos_Espaciais_Deriv_por_Elo( ...
    posicao_articular_atual, ...
    matrizes_homogeneas, ...
    COM_elos_juntas_ferramenta_local, ...
    joint_axes)

%#codegen

% -------------------------------------------------------------------------
% Entradas:
%   posicao_articular_atual            [3x1]
%   matrizes_homogeneas                [4x4x5]
%   COM_elos_juntas_ferramenta_local   [3x3]
%   joint_axes                         [3x3]
%
% Saída:
%   JacobianosEspaciais_deriv_porElo   [6x1x3]
% -------------------------------------------------------------------------

numero_elos = 3;
JacobianosEspaciais_deriv_porElo = zeros(6,1,numero_elos);

incremento_delta_q = 1e-6;

q_base = posicao_articular_atual(:);

% -------------------------------------------------------------------------
% Calcula Jacobianos espaciais no q + Δq e q - Δq
% -------------------------------------------------------------------------
for indice_elo = 1:numero_elos
    
    Jacobiano_plus = zeros(6,1);
    Jacobiano_minus = zeros(6,1);
    
    for indice_junta = 1:3
        
        vetor_incremento = zeros(3,1);
        vetor_incremento(indice_junta) = incremento_delta_q;
        
        % -------------------------------------------------------------
        % q_plus
        % -------------------------------------------------------------
        q_plus = q_base + vetor_incremento;
        T_plus = Simulink_Cinematica_Direta(q_plus);
        
        J_plus = Simulink_Jacobianos_Espaciais_por_Elo( ...
                    T_plus, ...
                    COM_elos_juntas_ferramenta_local, ...
                    joint_axes);
        
        Jacobiano_plus = J_plus(:,:,indice_elo);
        
        % -------------------------------------------------------------
        % q_minus
        % -------------------------------------------------------------
        q_minus = q_base - vetor_incremento;
        T_minus = Simulink_Cinematica_Direta(q_minus);
        
        J_minus = Simulink_Jacobianos_Espaciais_por_Elo( ...
                     T_minus, ...
                     COM_elos_juntas_ferramenta_local, ...
                     joint_axes);
        
        Jacobiano_minus = J_minus(:,:,indice_elo);
        
        % -------------------------------------------------------------
        % Derivada
        % -------------------------------------------------------------
        JacobianosEspaciais_deriv_porElo(:,:,indice_elo) = ...
            (Jacobiano_plus - Jacobiano_minus) / (2*incremento_delta_q);
    end
end
end
