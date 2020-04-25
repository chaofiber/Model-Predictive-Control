% Init
clear all
close all
addpath(genpath(cd));
addpath('C:\Users\nicha\Downloads\FORCES_client');
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
param = compute_controller_base_parameters;
T0_1 = param.T_sp+[3;1;0];
T0_2 = param.T_sp+[-1;-0.3;-4.5];
T0_3 = [12;12;12];
%[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);
%[T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);
%[T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);
%[T, p] = simulate_truck(T0_1, @controller_mpc_3, scen1);
%[T, p] = simulate_truck(T0_3, @controller_mpc_4, scen1);
%[T, p] = simulate_truck(T0_1, @controller_mpc_3, scen2);
%[~,~,t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);
[~,~,t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);