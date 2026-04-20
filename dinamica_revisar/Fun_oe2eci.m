function [rECI, vECI] = Fun_oe2eci(mu, a, e, inc, RAAN, argp, nu)
% OE2ECI  Converte elementos orbitais clássicos para ECI (r, v).
% Inputs:
%   mu   [SI]  parâmetro gravitacional
%   a    [SI]  semi-eixo maior
%   e          excentricidade
%   inc  [rad] inclinação
%   RAAN [rad] longitude do nodo ascendente (Ω)
%   argp [rad] argumento do perigeu (ω)
%   nu   [rad] anomalia verdadeira (ν)
%
% Outputs:
%   rECI [3x1] posição ECI
%   vECI [3x1] velocidade ECI

    p = a*(1 - e^2);
    r = p/(1 + e*cos(nu));
    % PQW
    r_pf = [r*cos(nu); r*sin(nu); 0];
    v_pf = sqrt(mu/p) * [-sin(nu); e + cos(nu); 0];

    % Matriz de rotação PQW->ECI
    cO = cos(RAAN); sO = sin(RAAN);
    ci = cos(inc);  si = sin(inc);
    cw = cos(argp); sw = sin(argp);

    C = [ cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,  sO*si;
          sO*cw + cO*sw*ci,  -sO*sw + cO*cw*ci, -cO*si;
          sw*si,              cw*si,             ci    ];

    rECI = C * r_pf;
    vECI = C * v_pf;
end
