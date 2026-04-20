function Jacob_linear = TESTE_Simulink_Jacobiana_Linear(matrizes_homogeneas)
%#codegen
NJ = 3;
Jacob_linear = zeros(3,NJ);

if ndims(matrizes_homogeneas) ~= 3 || size(matrizes_homogeneas,1) ~= 4 || ...
   size(matrizes_homogeneas,2) ~= 4 || size(matrizes_homogeneas,3) < (NJ+1)
    return;
end

p_ee = matrizes_homogeneas(1:3,4,end);

for j = 1:NJ
    idx = j; % precisa ser: idx=1 -> frame0 (T_00)
    Rprev = matrizes_homogeneas(1:3,1:3,idx);
    Oprev = matrizes_homogeneas(1:3,4,idx);

    if j == 1
        eixo = Rprev(:,3); % Z0
    else
        eixo = Rprev(:,1); % X_{j-1}
    end
    ne = norm(eixo);
    if ne > 1e-9, eixo = eixo/ne; end

    Jacob_linear(:,j) = cross(eixo, p_ee - Oprev);
end
end
