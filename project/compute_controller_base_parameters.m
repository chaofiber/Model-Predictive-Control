function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    % (1) modeling (dummy task)
    m3 = truck.m3;
    m2 = truck.m2;
    m1 = truck.m1;
    a3o = truck.a3o;
    a2o = truck.a2o;
    a1o = truck.a1o;
    a23 = truck.a23;
    a12 = truck.a12;
    To = truck.To;
    Ac = [(-a12-a1o)/m1,            a12/m1,             0;
                 a12/m2, (-a12-a23-a2o)/m2,        a23/m2;
                      0,            a23/m3, (-a23-a3o)/m3];
    Bc = [1/m1,    0;
             0, 1/m2;
             0,    0];
    Bcd = diag([1/m1, 1/m2, 1/m3]);
    dc = truck.w + [a1o*To; a2o*To;a3o*To];
    % (2) discretization
    Ts = 60;
    %A = eye(3) + Ts*Ac;
    %B = Ts*Bc;
    %Bd = Ts*Bcd;
    [A,BBd] = c2d(Ac,[Bc, Bcd],Ts);
    B = BBd(:,1:2);
    Bd = BBd(:,3:5);
    
    % (3) set point computation (Based on the discretized?)
    % T(1)=A(1,1)*T(1)+A(1,2)*T(2)+A(1,3)*T(3)+B(1,1)*P(1)+B(1,2)*p(2)+Bd(1,:)*dc;
    % T(2)=A(2,1)*T(1)+A(2,2)*T(2)+A(2,3)*T(3)+B(2,1)*P(1)+B(2,2)*p(2)+Bd(2,:)*dc;
    % T(3)=A(3,1)*T(1)+A(3,2)*T(2)+A(3,3)*T(3)+B(3,1)*P(1)+B(3,2)*p(2)+Bd(3,:)*dc;
    % Fx=e to solve for x=[T(3),p(1),p(2)]';
    %F = [  A(1,3),   B(1,1), B(1,2);
    %       A(2,3),   B(2,1), B(2,2);
    %     A(3,3)-1, B(3,1), B(3,2)];
    %e = [-21-A(1,1)*(-21)-A(1,2)*0.3-Bd(1,:)*dc;
    %    0.3-A(2,1)*(-21)-A(2,2)*0.3-Bd(2,:)*dc;
    %     -A(3,1)*(-21)-A(3,2)*0.3-Bd(3,:)*dc];
    %sol = linsolve(F,e);
    
    %T_sp = [-21; 0.3; sol(1)];
    %p_sp = [sol(2); sol(3)];
    T_sp = [-21; 0.3; -(Ac(3,2)*0.3+Bcd(3,3)*dc(3))/Ac(3,3)];
    p_sp = [                -(Ac(1,1)*(-21)+Ac(1,2)*0.3+Bcd(1,1)*dc(1))*m1;
            -(Ac(2,1)*(-21)+Ac(2,2)*0.3+Ac(2,3)*T_sp(3)+Bcd(2,2)*dc(2))*m2];
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - p_sp;
    Xcons = Tcons - T_sp;
    
    % (5) LQR cost function
    Q = [500,0,0;
         0,100,0;
         0, 0, 0];
    R = [0.01,0
         0, 0.001];
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    
    % compute and add other properties
    [P,K,~] = idare(A, B, Q, R,[],[]);
    param.P = P;
    param.K = K;  % the controller should be u = -K * x
end

