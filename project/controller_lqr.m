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

% This is an infinite LQR controller, 
% J(x(0))=min_U \sum_{i=0}^{\infty}(x_i^TQx_i+u_i^Tu_i)
% We need to compute P_inf=X by Algebraic Riccati Equation

p = -param.K*(T-param.T_sp)+param.p_sp;
end

function param = init()
param = compute_controller_base_parameters;
A = param.A;
B = param.B;
Q = param.Q;
R = param.R;
[P,K,~] = idare(A, B, Q, R,[],[]);
param.K = K;
param.P = P;
disp(P);
disp(eig(P));
end