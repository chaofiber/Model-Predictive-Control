% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
param = compute_controller_base_parameters;
T0_1 = param.T_sp+[3;1;0];
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);