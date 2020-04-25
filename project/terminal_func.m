clear;
clc;
T0_2 = [-21.0000;0.3000;7.3200]+[-1;-0.3;-4.5];
load('system/parameters_scenarios.mat');

sum = 0;
for i=1:1:20
    [~,~,t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
    sum = sum + t_sim_forces;
end

sum_ = 0;
for i=1:1:20
    [~,~,t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);
    sum_ = sum_ + t_sim;
end




disp('Average sim_forces');disp(sum/20);
disp('Average sim');disp(sum_/20);