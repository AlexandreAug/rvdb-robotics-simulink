function Jb_i = Fun_JacobianBasePorElo(T0i)
% Retorna Jacobiano (6x6) que mapeia torção da base (Vb=[v_b; w_b]) para a torção no frame do elo i (no ponto do CoM do elo i)
% T0i: (4x4) homogênea do frame do elo i em relação ao frame 0 (base)
% Convenção LVLH/Fehse: usar T0i coerente com sua cinemática (R0i, p0i)
    R0i = T0i(1:3,1:3);
    p0i = T0i(1:3,4);
    % Transformação de movimento: v_i = X_i0 * v_0
    Jb_i = Fun_SpatialXform(R0i.', -R0i.'*p0i); % X_i0 = [R_i0 0; skew(p_i0)*R_i0 R_i0], com R_i0=R0i'
end
