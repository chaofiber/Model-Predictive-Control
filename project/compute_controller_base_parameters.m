function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    % (2) discretization
    Ts = 60;
    A = ...
    B = ...
    
    % (3) set point computation
    T_sp = ...
    p_sp = ...
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = ...
    Xcons = ...
    
    % (5) LQR cost function
    Q = ...
    R = ...
    
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
end

