% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_2(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%       warning('MPC infeasible');
% end
% p = ...;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip here, e.g.
% N = 30;
% nx = size(param.A,1);
% nu = size(param.B,2);
%
% U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
%
% objective = 0;
% constraints = [...];
% for k = 1:N-1
%   constraints = [constraints, ...];
%   objective = objective + ... ;
% end
% objective = objective + ... ;
%
% ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
% yalmip_optimizer = optimizer(constraints,objective,ops,... , ... );
end