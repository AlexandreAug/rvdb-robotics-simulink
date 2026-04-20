function robot = rigidBodyTree_chaser()
    robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 6);

    % --- Parâmetros DH (extraídos do seu Simulink_ForwardKinematics) ---
    % Ordem: [alpha, a, d, theta]
    dh_params = [
        0,      0,      0.175,  0;    % base → junta1
        pi/2,   0,      0,      0;    % junta1 → junta2
        0,      0.350,  0,      0;    % junta2 → junta3
        0,      0.325,  0,      0;    % junta3 → elo_fixo
        0,      0.150,  0,      0     % elo_fixo → ferramenta (EEF)
    ];

    % --- Nomes dos corpos ---
    bodyNames = {'corpo1', 'corpo2', 'corpo3'};
    parentNames = {'base', 'corpo1', 'corpo2'};

    % --- Propriedades dinâmicas combinadas (seus valores exatos) ---
    massas = [
        massa_junta1_elo1;
        massa_junta2_elo2;
        massa_junta3_elo3_ferramenta
    ];

    inercia = cat(3, ...
        tensorInercia_junta1_elo1, ...
        tensorInercia_junta2_elo2, ...
        tensorInercia_junta3_elo3_ferramenta);

    coms = [
        COM_junta1_elo1, ...
        COM_junta2_elo2, ...
        COM_junta3_elo3_ferramenta
    ];

    % --- Montar o robô ---
    for i = 1:3
        body = rigidBody(bodyNames{i});
        
        % Junta revolute
        joint = rigidBodyJoint(['j', num2str(i)], 'revolute');
        
        % Transformação DH (alpha, a, d, theta=0 → theta é variável)
        alpha = dh_params(i,1); a = dh_params(i,2); d = dh_params(i,3);
        T_dh = dhparams(d, a, alpha, 0);
        setFixedTransform(joint, T_dh, 'dh');
        
        body.Joint = joint;
        
        % Propriedades dinâmicas
        body.Mass = massas(i);
        body.Inertia = inercia(:,:,i);
        body.CenterOfMass = coms(:,i);
        
        % Adicionar ao robô
        if i == 1
            addBody(robot, body, 'base');
        else
            addBody(robot, body, parentNames{i});
        end
    end

    % --- Elo fixo + ferramenta (corpo rígido fixo) ---
    body_fixo = rigidBody('ferramenta');
    joint_fixo = rigidBodyJoint('fix_ferramenta', 'fixed');
    
    % DH do elo fixo + ferramenta
    T_fixo = dhparams(0, 0.325+0.150, 0, 0);  % a = 0.325 + 0.150 = 0.475 m
    setFixedTransform(joint_fixo, T_fixo, 'dh');
    
    % Já está incluso no corpo3 (massa_junta3_elo3_ferramenta)
    body_fixo.Joint = joint_fixo;
    body_fixo.Mass = 0;  % já incluso
    addBody(robot, body_fixo, 'corpo3');

    % --- EEF name ---
    robot.BodyNames{end} = 'eef';  % nome do end-effector
end

% =========================================================================
% Função auxiliar: dhparams
% =========================================================================
function T = dhparams(d, a, alpha, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [ct,        -st*ca,     st*sa,      a*ct;
         st,         ct*ca,     -ct*sa,      a*st;
         0,          sa,         ca,          d;
         0,          0,          0,           1];
end