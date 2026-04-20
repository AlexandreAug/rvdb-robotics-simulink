%% Fun_Euler2Quat.m
function q = Fun_Euler2Quat(euler0_deg)
% Sequência 3-2-1 (yaw Z, pitch Y, roll X) em GRAUS.
e     = deg2rad(euler0_deg(:)');
psi   = e(1); % yaw (Z)
theta = e(2); % pitch (Y)
phi   = e(3); % roll (X)

c1 = cos(phi/2);   s1 = sin(phi/2);
c2 = cos(theta/2); s2 = sin(theta/2);
c3 = cos(psi/2);   s3 = sin(psi/2);

w = c1*c2*c3 + s1*s2*s3;
x = s1*c2*c3 - c1*s2*s3;
y = c1*s2*c3 + s1*c2*s3;
z = c1*c2*s3 - s1*s2*c3;

q = [w; x; y; z];
q = q / norm(q);
end
