function dq_ref = Simulink_Cinematica_Inversa_Posicao_Alinhamento( ...
    pos_desejada_0, pos_atual_0, Jv, Jw, eixo_ins_0, radial0, ...
    Kp_pos, Kp_ori, limite_dq, lambda)
%#codegen
% IK 3R: posição + alinhamento de eixo (eixo_ins_0 || radial0), tudo em {0}.
% Entradas:
%   pos_desejada_0, pos_atual_0 : [3x1]
%   Jv, Jw                      : [3x3]
%   eixo_ins_0                  : [3x1] eixo de inserção da ferramenta em {0} (unitário ou não)
%   radial0                     : [3x1] eixo radial em {0} (unitário ou não)
%   Kp_pos, Kp_ori              : ganhos (escalares)
%   limite_dq                   : escalar ou [3x1]
%   lambda                      : damping (>= 1e-3)

dq_ref = zeros(3,1);

% reshape/força dimensões
pos_desejada_0 = pos_desejada_0(:);
pos_atual_0    = pos_atual_0(:);
eixo_ins_0     = eixo_ins_0(:);
radial0        = radial0(:);

Jv = reshape(Jv,[3,3]);
Jw = reshape(Jw,[3,3]);

% proteção numérica
if any(~isfinite([pos_desejada_0; pos_atual_0; eixo_ins_0; radial0])) || ...
   any(~isfinite(Jv(:))) || any(~isfinite(Jw(:)))
    return;
end

% erro de posição
e_pos = pos_desejada_0 - pos_atual_0;

% normaliza eixos
u = eixo_ins_0;
nu = norm(u);
if nu < 1e-12
    u = [0;0;1];
else
    u = u / nu;
end

n = radial0;
nn = norm(n);
if nn < 1e-12
    n = [0;0;1];
else
    n = n / nn;
end

% if dot(u, n) < 0
%     u = -u;
% end

% erro de alinhamento (pequeno ângulo): u -> n
e_ori = cross(u, n);   % 3x1

% sinais desejados (velocidades cartesianas virtuais)
v_d = Kp_pos * e_pos;  % 3x1
w_d = Kp_ori * e_ori;  % 3x1

y = [v_d; w_d*0.5];        % 6x1
J = [Jv; Jw];          % 6x3

% DLS geral: dq = J' (J J' + lambda^2 I)^-1 y
if lambda < 1e-3, lambda = 1e-3; end
lam2 = lambda^2;

JJt = J * J.';                                 % 6x6
dq  = J.' * ((JJt + lam2*eye(6)) \ y);         % 3x1

% saturação
if isscalar(limite_dq)
    lim = abs(limite_dq)*ones(3,1);
else
    lim = abs(limite_dq(:));
    if numel(lim) ~= 3
        lim = lim(1)*ones(3,1);
    end
end
dq = max(min(dq, lim), -lim);

dq_ref = dq;
end
