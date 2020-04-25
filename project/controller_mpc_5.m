% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer Est

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
% initial estimation
if isempty(Est)
    Est = [T;120;1200;3600];
end
T_est = Est(1:3);
d_est = Est(4:6);
equ = param.Est * [-param.Bd * d_est; param.r];  
[u_mpc,errorcode] = yalmip_optimizer([T_est-equ(1:3);equ]);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc + equ(4:5);	
% the real evolution: x(k+1) = Ax(k) + B*p + Bd *d;
% update current temperature and disturbance estimation
Est = param.A_aug * Est + param.B_aug*p + param.L*(T - param.C_aug*Est);
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object
param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip here, e.g.
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
equ = sdpvar(5,1);

objective = 0;
Xcons = param.Tcons - [equ(1:3),equ(1:3)];
Ucons = param.Pcons - [equ(4:5),equ(4:5)];
constraints = Xcons(:,1) <= X{1} <= Xcons(:,2);
for k = 1:N-1
  constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
  constraints = [constraints, Xcons(:,1) <= X{k+1} <= Xcons(:,2)];
  constraints = [constraints, Ucons(:,1) <= U{k} <= Ucons(:,2)];
  objective = objective +  X{k}'*param.Q*X{k} + U{k}'*param.R*U{k};
end

% terimical constraints x(N)\in X_{LQR}
[Ax, bx] = compute_X_LQR;
constraints = [constraints, Ax*X{N}<=bx];
objective = objective + X{N}'*param.P*X{N};

ops = sdpsettings('verbose',0,'solver','quadprog');
fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops,[X{1};equ], U{1} );
end