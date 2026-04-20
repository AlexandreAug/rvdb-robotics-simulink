function Jacobiana_angular = TESTE_Simulink_Jacobiana_Angular(matrizes_homogeneas)
%#codegen
% Jacobiano Angular para 3R Z–X–X
% Junta 1: eixo Z do frame 0
% Junta 2: eixo X do frame 1
% Junta 3: eixo X do frame 2
% Cada eixo expresso em {0}.

NJ = 3;

Jacobiana_angular = zeros(3,NJ);

% Proteção mínima
if ndims(matrizes_homogeneas) ~= 3 || size(matrizes_homogeneas,1) ~= 4 || ...
   size(matrizes_homogeneas,2) ~= 4 || size(matrizes_homogeneas,3) < (NJ+1)
    return;
end

for j = 1:NJ
    % frame ANTERIOR à junta j:
    % j=1 -> frame0 (slice 1)
    % j=2 -> frame1 (slice 2)
    % j=3 -> frame2 (slice 3)
    idx = j;

    Rprev = matrizes_homogeneas(1:3,1:3,idx);

    if j == 1
        eixo = Rprev(:,3);   % Z do frame anterior
    else
        eixo = Rprev(:,1);   % X do frame anterior
    end

    ne = norm(eixo);
    if ne > 1e-9, eixo = eixo/ne; end

    Jacobiana_angular(:,j) = eixo;
end

end
