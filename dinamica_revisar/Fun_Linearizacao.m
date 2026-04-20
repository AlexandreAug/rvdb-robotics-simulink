% ========================================================================
% Fun_Linearizacao.m — Linearização numérica do modelo dinâmico
% ========================================================================
% Calcula A = ∂f/∂x , B = ∂f/∂u  por diferenças finitas
% ========================================================================
function [A,B,eigA] = Fun_Linearizacao(x_eq,u_eq,params)

    nx = length(x_eq);
    nu = length(u_eq);
    h  = 1e-6;                     % passo de perturbação
    f0 = MR_DynamicsODE(0,x_eq,params);

    % --- Matriz A --------------------------------------------------------
    A = zeros(nx);
    for i = 1:nx
        x1 = x_eq; x1(i) = x1(i)+h;
        f1 = MR_DynamicsODE(0,x1,params);
        A(:,i) = (f1 - f0)/h;
    end

    % --- Matriz B --------------------------------------------------------
    B = zeros(nx,nu);
    for j = 1:nu
        u1 = u_eq; u1(j) = u1(j)+h;
        p2 = params; 
        p2.torque_func = @(t,q,dq,vb,wb) deal(zeros(3,1),zeros(3,1),u1);
        f1 = MR_DynamicsODE(0,x_eq,p2);
        B(:,j) = (f1 - f0)/h;
    end

    eigA = eig(A);
end
