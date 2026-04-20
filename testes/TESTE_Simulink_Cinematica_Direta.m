function homogeneas = TESTE_Simulink_Cinematica_Direta(juntas)
%#codegen

juntas = juntas(:);
j1 = juntas(1); j2 = juntas(2); j3 = juntas(3);

Rx = @(t)[1 0 0 0;
          0 cos(t) -sin(t) 0;
          0 sin(t)  cos(t) 0;
          0 0 0 1];

Rz = @(t)[cos(t) -sin(t) 0 0;
          sin(t)  cos(t) 0 0;
          0 0 1 0;
          0 0 0 1];

Ty = @(d)[1 0 0 0;
          0 1 0 d;
          0 0 1 0;
          0 0 0 1];

Tz = @(d)[1 0 0 0;
          0 1 0 0;
          0 0 1 d;
          0 0 0 1];

homogeneas = zeros(4,4,5);

T = eye(4);
homogeneas(:,:,1) = T;                  % frame0

T = T * Tz(0.05) * Rz(j1) * Tz(0.10);   % junta1 + elo1
homogeneas(:,:,2) = T;

T = T * Ty(0.05) * Rx(j2) * Ty(0.30);   % junta2 + elo2
homogeneas(:,:,3) = T;

T = T * Ty(0.05) * Rx(j3) * Ty(0.30);   % junta3 + elo3
homogeneas(:,:,4) = T;

T = T * Ty(0.15);                       % ferramenta
homogeneas(:,:,5) = T;
end
