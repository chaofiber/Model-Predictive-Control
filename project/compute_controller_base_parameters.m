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
    param.dc = dc; 
    
    % (2) discretization
    Ts = 60;
    [A,BBd] = c2d(Ac,[Bc, Bcd],Ts);
    B = BBd(:,1:2);
    Bd = BBd(:,3:5);
    
    % (3) set point computation 
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
    
    % LQR
    [P,K,~] = idare(A, B, Q, R,[],[]);
    param.P = P;
    param.K = K;  % the LQR controller should be u = -K * x
    
    % soft constraints
    param.v = 1000;
    
    % augmented model
    param.A_aug = [A,    Bd;
             zeros(3),eye(3)];
    param.B_aug = [B; zeros(3,2)];
    param.C_aug = [eye(3), zeros(3)];
    param.L = place(param.A_aug',param.C_aug',[0.55;0.59;0.51;0.48;0.47;0.52]);
    param.L = param.L';
    
    % observor z = Hy=H * x
    H = [1 0 0;
         0 1 0];
    Est_inv = [A - eye(3), B;
               H*eye(3) , zeros(2,2)];
           
    param.Est = Est_inv^(-1);
    param.Bd = Bd;
    param.r = T_sp(1:2);
end

