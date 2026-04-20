%% Script de Diagnóstico de Cinemática e Singularidade
clear; clc;

% 1. Parâmetros do Robô (baseado no seu código)
juntas_teste = [0; 45; 45] * pi/180; % Configuração de teste (em radianos)
lambda = 0.01; % Damping que você está usando

% 2. Chamar sua cinemática direta
% (Certifique-se que suas funções estão no mesmo caminho ou pasta)
try
    H = TESTE_Simulink_Cinematica_Direta(juntas_teste);
    Jv = TESTE_Simulink_Jacobiana_Linear(H);
    Jw = TESTE_Simulink_Jacobiana_Angular(H);
catch
    error('Certifique-se que as funções Simulink_Cinematica_Direta, Linear e Angular estão acessíveis.');
end

% 3. Análise da Matriz Jacobiana Linear (3x3)
det_Jv = det(Jv);
cond_Jv = cond(Jv);

fprintf('--- Análise da Posição (3x3) ---\n');
fprintf('Determinante de Jv: %.4f\n', det_Jv);
fprintf('Número de Condicionamento: %.4f (Ideal < 100)\n', cond_Jv);

if cond_Jv > 1e3
    fprintf('ALERTA: O braço está perto de uma SINGULARIDADE de posição!\n');
end

% 4. O Problema do Rank (6x3)
J_completo = [Jv; Jw];
rank_J = rank(J_completo);

fprintf('\n--- Análise da Tarefa Completa (6x3) ---\n');
fprintf('Posto (Rank) da matriz 6x3: %d\n', rank_J);
if rank_J < 6
    fprintf('DAGNÓSTICO: Você tem apenas %d DOFs. Tentar controlar 6 variáveis gera conflito.\n', rank_J);
end

% 5. Teste de Inversão Damped Least Squares (DLS)
% Simulando um erro de posição (v_d) e um erro de orientação (w_d)
v_d = [0.1; 0.05; 0.05]; % erro de 10cm em X, 5cm em Y...
w_d = [0.1; 0; 0];        % erro de alinhamento

% Cenário A: Tudo junto (6x3) - O que você tem hoje
y_tudo = [v_d; w_d];
dq_tudo = J_completo' * pinv(J_completo * J_completo' + lambda^2 * eye(6)) * y_tudo;

% Cenário B: Somente Posição (3x3) - Mais estável
dq_pos = Jv \ v_d;

fprintf('\n--- Comparação de Velocidades de Junta (dq) ---\n');
fprintf('dq (Tarefa 6x1) -> J1: %.4f, J2: %.4f, J3: %.4f\n', dq_tudo);
fprintf('dq (Tarefa 3x1) -> J1: %.4f, J2: %.4f, J3: %.4f\n', dq_pos);

% Verificação de "Explosão"
if max(abs(dq_tudo)) > 10 * max(abs(dq_pos))
    fprintf('\nCONCLUSÃO: O erro de orientação está inflando as velocidades de junta!\n');
    fprintf('Isso causa o movimento "louco" no simulador.\n');
end