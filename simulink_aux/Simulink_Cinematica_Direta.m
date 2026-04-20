
function matrizes_homogeneas = Simulink_Cinematica_Direta(juntas_atual)
% Entradas:
%   posição atual das juntas [3x1] = [q1; q2; q3] (rad)
% Saída:
%   matrizes_homogeneas [4x4x5] = homogêneas acumuladas até a ferramenta
% =========================================================================

% --- proteção contra tamanho incorreto ---
if numel(juntas_atual) ~= 3
    matrizes_homogeneas = repmat(eye(4), [1 1 5]);
    return;
end

% --- conversão para graus ---
q1 = juntas_atual(1);
q2 = juntas_atual(2);
q3 = juntas_atual(3);

DH = [  0,    0.000,  0.175,  q1; % junta 1
        pi/2, 0.000,  0.000,  q2; % junta 2
         0,   0.350,  0.000,  q3; % junta 3
         0,   0.325,  0.000,  0;
         0,   0.150,  0.000,  0];

% --- função homogênea (Craig Eq. 3.6) ---
A_fun = @(alpha,a,d,theta) [ ...
    cos(theta), -sin(theta), 0, a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
    sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d;
    0, 0, 0, 1];

% --- inicialização ---
n = size(DH,1); % = 5
T = eye(4);
T(1:3,4) = [0; -0.125; 0];   % deslocamento físico da junta 1 em -y da face superior da base física
matrizes_homogeneas = zeros(4,4,n);

% --- cálculo iterativo ---
for i = 1:n
    alpha = DH(i,1);
    a     = DH(i,2);
    d     = DH(i,3);
    theta = DH(i,4);
    A = A_fun(alpha, a, d, theta);
    T = T * A;
    matrizes_homogeneas(:,:,i) = T;
end
end
