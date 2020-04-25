% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.
    N = 30;
    A = param.A;
    B = param.B;
    Q = param.Q;
    R = param.R;
    [~,K,~] = idare(A, B, Q, R,[],[]);
    L = A-B*K; % closed-loop system x(k+1)=L*x(k)
    A_xx = []; % constraints resulting from states x0 to x(N)
    A_xu = []; % constraints resulting from inputs u0 to u(N-1)
    A_xx = [A_xx;eye(3)];
    for i=1:1:N
        A_xx = [A_xx;L^i];
        A_xu = [A_xu;-K*L^(i-1)];
    end
    A_x = [A_xx;-A_xx;A_xu;-A_xu];
    b_x = [repmat(param.Xcons(:,2),(N+1),1);-repmat(param.Xcons(:,1),N+1,1);...
           repmat(param.Ucons(:,2),N,1);-repmat(param.Ucons(:,1),N,1)];
       
    Options.FaceAlpha=0.1;
    %Options.color='g';
    P = polytope(A_x,b_x);
    figure(2);
    plot(P,Options);
    hold on;
    plot3(-1,-0.3,-4.5,'.','MarkerSize',10);
    hold on;
    plot3(0,0,0,'.','MarkerSize',10);
    legend("feasible set","infeasible initialization");
end

