% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
% p = ...;

% This is an infinite LQR controller, 
% J(x(0))=min_U \sum_{i=0}^{\infty}(x_i^TQx_i+u_i^Tu_i)
% We need to compute P_inf=X by Algebraic Riccati Equation
A = param.A;
B = param.B;
Q = param.Q;
R = param.R;
[~,K,~] = idare(A, B, Q, R,[],[]);
%p = -((B')*X*B+R)^(-1)*(B')*X*A*T;
p = -K*(T-param.T_sp);
end

function param = init()
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
% param.F = ...,

end