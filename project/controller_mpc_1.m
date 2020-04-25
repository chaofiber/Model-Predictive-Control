% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
[u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = param.p_sp + u_mpc;
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

objective = 0;
constraints = param.Xcons(:,1) <= X{1} <= param.Xcons(:,2); % Init constraint, sometimes do not enforce it
for k = 1:N-1
  constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
  constraints = [constraints, param.Xcons(:,1) <= X{k+1} <= param.Xcons(:,2)]; 
  constraints = [constraints, param.Ucons(:,1) <= U{k} <= param.Ucons(:,2)];
  objective = objective + X{k}'*param.Q*X{k} + U{k}'*param.R*U{k}; 
end
objective = objective + X{N}'*param.P*X{N}; % add terminal cost

ops = sdpsettings('verbose',0,'solver','quadprog');
fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops,X{1},U{1} );
end