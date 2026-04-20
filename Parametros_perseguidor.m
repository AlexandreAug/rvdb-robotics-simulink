%% Parametros_perseguidor.m
% clear;
clc; 

%% ===== Base geral =====
%% Centro de massa
COM_baseGeral = [0; 0; 0] / 100;

%% Tensor de inércia
tensorInercia_baseGeral = ...
[7.9480E+05 6.6007E-09 1.6386E-05;
 6.6007E-09 7.9480E+05 -1.2573E-09;
 1.6386E-05 -1.2573E-09 1.4485E+06] * 1e-7;

%% Parâmetros físicos
comprimento_baseGeral = 10 / 100;
raio_baseGeral        = 25 / 100;
diametro_baseGeral    = 2 * raio_baseGeral;

global massa_baseGeral
massa_baseGeral = 3652.9556 / 1000;

%% ===== Junta 1 =====
%% Centro de massa
COM_junta1 = [0; -12.5; 7.5] / 100;

%% Tensor de inércia
tensorInercia_junta1 = ...
[395.7537 0 0;
 0 395.7537 0;
 0 0 339.2174] * 1e-7;

%% Parâmetros físicos
comprimento_junta1 = 5 / 100;
raio_junta1        = 5 / 100;
diametro_junta1    = 2 * raio_junta1;
massa_junta1       = 71.8345 / 1000;

%% ===== Junta 2 ====
%% Centro de massa
COM_junta2 = [0; -12.5; 7.5] / 100;

%% Tensor de inércia
tensorInercia_junta2 = ...
[339.2174 0 0;
 0 395.7537 0;
 0 0 339.7537] * 1e-7;

%% Parâmetros físicos
comprimento_junta2 = comprimento_junta1;
raio_junta2        = raio_junta1;
diametro_junta2    = 2 * raio_junta2;
massa_junta2       = massa_junta1;

%% ===== Junta 3 =====
%% Centro de massa
COM_junta3 = [0; 22.5; 22.5] / 100;

%% Tensor de inércia
tensorInercia_junta3 = ...
[339.2174	  0.0000     0.0000;
   0.0000   395.7537     0.0000;
   0.0000	  0.0000   395.7537] * 1e-7;

%% Parâmetros físicos
comprimento_junta3 = comprimento_junta1;
raio_junta3        = raio_junta1;
diametro_junta3    = 2 * raio_junta3;
massa_junta3       = massa_junta1;

%% ===== Elo 1 =====
%% Centro de massa
COM_elo1 = [0; -12.5; 15] / 100;

%% Tensor de inércia
tensorInercia_elo1 = ...
[1661.8171         0.0000     5.7431E-07;
    0.0000      1661.8170     0.00000000;
    0   0.0000	624.087] * 1e-7;

%% Parâmetros físicos
comprimento_elo1 = 10 / 100;
raio_elo1        = 2.5 / 100;
diametro_elo1    = 2 * raio_elo1;
massa_elo1       = 122.1981 / 1000;

%%  ===== Elo 2 =====
%% Centro de massa
COM_elo2 = [0; 5; 22.5] / 100;

%% Tensor de inércia
tensorInercia_elo2 = ...
[28296.2947 0 1.8770E-10;
 0 1763.5648 0;
 1.8770E-10 0 28296.2951] * 1e-7;

%% Parâmetros físicos
comprimento_elo2 = 30 / 100;
raio_elo2        = raio_elo1;
diametro_elo2    = 2 * raio_elo2;
massa_elo2       = 323.6528 / 1000;

%%  ===== Elo 3 =====
%% Centro de massa
COM_elo3 = [0; 40; 22.5] / 100;

%% Tensor de inércia
tensorInercia_elo3= ...
[28296.2947 1.2547E-10 0;
 1.2547E-10 1763.5648 0;
 0 0 28296.2951] * 1e-7;

%% Parâmetros físicos
comprimento_elo3 = comprimento_elo2;
raio_elo3        = raio_elo1;
diametro_elo3    = 2 * raio_elo3;
massa_elo3       = massa_elo2;

comprimento_elos = [comprimento_elo1;comprimento_elo2; comprimento_elo3];

%% ===== Ferramenta =====
%% Centro de massa
COM_ferramenta = [0; 62.5; 22.5] / 100;

%% Tensor de inércia
tensorInercia_ferramenta = ...
[811.3326     0.0000	    0.0000;
  0.0000     16.2854     0.0000;
  0.0000      0.0000   811.3326] * 1e-7;

%% Parâmetros físicos
comprimento_ferramenta = 15 / 1e2;
raio_ferramenta        = 0.75 / 100;
diametro_ferramenta    = 2 * raio_ferramenta;
massa_ferramenta       = 40.8211 / 1000;

%% ===== Combinados
% Junta 1 + Elo 1
COM_junta1_elo1 = [0; -12.5; 12.2234] / 100;

tensorInercia_junta1_elo1 = ...
[4602.3219 0 -1.0352E-07;
 0 4602.3221 0;
 -1.0352E-07 0 963.3044] * 1e-7;

massa_junta1_elo1 = massa_junta1 + massa_elo1;

% ---------------------------------------------------
% Junta 2 + Elo 2
COM_junta2_elo2 = [0; 1.8214; 22.5] / 100;

tensorInercia_junta2_elo2 = ...
[46638.9686 -4.4399E-06 3.1218E-10;
 -4.4399E-06 2159.3182 -1.4921E-10;
 3.1218E-10 -1.4921E-10 46695.5045] * 1e-7;

massa_junta2_elo2 = massa_junta2 + massa_elo2;

% ---------------------------------------------------
% Junta 3 + Elo 3 + Ferramenta
COM_junta3_elo3_ferramenta = [0; 39.2239; 22.5] / 100;

tensorInercia_junta3_elo3_ferramenta = ...
[71849.0024 -3.7771E-06 0;
 -3.7771E-06 2175.6035 -7.6022E-07;
 0 -7.6022E-07 71905.5383] * 1e-7;

massa_junta3_elo3_ferramenta = massa_junta3 + massa_elo3 + massa_ferramenta;

%% ========== Espaçonave chaser ==========
%% Chaser Pose Home (Modo = 0; MR Não operacional)
COM_chaser_home = [0; 3.2933; 4.507] / 100;

tensorInercia_chaser_home = ...
[2.1098E+06 -7.6447E-09 -5.8552E-09;
-7.6447E-09 1.3435E+06 -3.6232E+05;
-5.8552E-09 -3.6232E+05 2.2202E+06] * 1e-7;

%% Parâmetros físicos do chaser
%% Massa vetor do chaser
massa_vetor_total_chaser    = [massa_baseGeral, ...
                               massa_elo1, massa_elo2, massa_elo3 ...
                               massa_junta1, massa_junta2, massa_junta3, ...
                               massa_ferramenta];

massa_vetor_elos            = [massa_elo1, massa_elo2, massa_elo3];

massa_vetor_juntas          = [massa_junta1, massa_junta2, massa_junta3];

massas_elos_juntas_ferramenta = [massa_junta1_elo1; ...
                                 massa_junta2_elo2; ...
                                 massa_junta3_elo3_ferramenta];


%% Massa escalar do chaser
massa_escalar_total_chaser = sum(massa_vetor_total_chaser); % 4.6788
massa_escalar_elos         = sum(massa_vetor_elos);
massa_escalar_juntas       = sum(massa_vetor_juntas);

massa_escalar_elos_juntas     = massa_escalar_elos + massa_escalar_juntas;

massa_escalar_MR = massa_escalar_elos + massa_escalar_juntas + massa_ferramenta;

volume_chaser_m3 = 17328829 / 1e3;
densidade_kgm3   = 2.7 * 1000;

% Espessuras
% A base, os elos e a ferramenta são ocos, conforme:
parede_base       = 0.25 / 100;
parede_elo        = 0.25 / 100;
parede_ferramenta = 0.25 / 100;

%% Tensores de inercia
tensorInercia_elos   = [tensorInercia_elo1; tensorInercia_elo2; tensorInercia_elo3];
tensorInercia_juntas = [tensorInercia_junta1, tensorInercia_junta2, tensorInercia_junta3];

tensorInercia_elos_juntas_ferramenta = cat(3, ...
                                        tensorInercia_junta1_elo1, ...
                                        tensorInercia_junta2_elo2, ...
                                        tensorInercia_junta3_elo3_ferramenta);

COM_elos_juntas_ferramenta = [COM_junta1_elo1, ...
                              COM_junta2_elo2, ...
                              COM_junta3_elo3_ferramenta];

%% Limites de rotação das juntas [rad]
limites_J1 = [-deg2rad(360); deg2rad(360)];
limites_J2 = [-deg2rad(90) ; deg2rad(90)];
limites_J3 = [-deg2rad(130); deg2rad(130)];

limitesRot_juntas = ...
[limites_J1, ...
 limites_J2, ...
 limites_J3]';

%% Limites de velocidade angular por junta [rad/s]
limVel_j1 = [0.1*pi]; 
limVel_j2 = [0.1*pi]; 
limVel_j3 = [0.1*pi];  

% Limite
limitesVel_juntas = ...
[limVel_j1, ...
 limVel_j2, ...
 limVel_j3]';

%% Limite nominal de torque das juntas (Nm)
limitesTorque_J1 = [-4; 4];
limitesTorque_J2 = [-4; 4];
limitesTorque_J3 = [-4; 4];

limitesTorque_juntas = ...
[limitesTorque_J1, ...
 limitesTorque_J2, ...
 limitesTorque_J3]';

%% Limite máximo de torque das juntas (Nm)
limitesTorque_max_juntas = limitesTorque_juntas * 1.05;