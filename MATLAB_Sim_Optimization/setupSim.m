%% Setup for 2.74 Final Project
clear; clc;

% Add paths
addpath(genpath('./codegen'));
addpath(genpath('./utilities'));

% Setup CasADi
if ismac
    addpath(genpath("./casadi/casadi_osx"));
elseif isunix
    addpath(genpath("./casadi/casadi_linux"));
elseif ispc
    addpath(genpath("./casadi/casadi_windows"));
end
import casadi.*

% Generate dynamics + load functions for leg
buildLegDynamics()
A_fn = casadi.Function.load('codegen/A_fn.casadi');
b_fn = casadi.Function.load('codegen/b_fn.casadi');
energy_fn = casadi.Function.load('codegen/energy_fn.casadi');
pos_end_effector = casadi.Function.load('codegen/pos_end_effector.casadi');
vel_end_effector = casadi.Function.load('codegen/vel_end_effector.casadi');
J_end_effector = casadi.Function.load('codegen/J_end_effector.casadi');
keypoints_fn = casadi.Function.load('codegen/keypoints_fn.casadi');

% Parameters for leg
m1 = 13.65*2*10^-3; % swing
m2 = 486.95 * 10^-3; % hip to knee 
m3 = 53.25 * 10^-3 + 0.1; % knee to foot (bottom leg)        
m4 = 53.25 * 10^-3 + 0.1; % hip to head (torso)
I1 = 0.00028640746; % swing
I2 = 5.11625 * 10^-4; % hip to knee
I3 = 2.8628975 * 10^-4; % knee to foot (bottom leg)
I4 = 2.8628975 * 10^-4; % hip to head (torso)
l_OA=.3;              l_DB=0.097; 
l_BC=0.127;              l_DE=0.127;
l_O_m1= l_OA / 2;           l_O_m2=l_OA; 
l_B_m3= l_BC / 2;          l_D_m4=l_DE / 2;
N = 18.75;
Ir = 0.0035/N^2;
g = 9.81;    

%% Parameter vector
params    = [m1 m2 m3 m4 I1 I2 I3 I4 Ir N l_O_m1 l_O_m2 l_B_m3 l_D_m4 l_OA l_DB l_BC l_DE g]';
q_max_val = [pi; pi/1.2; pi/1.2];
q_min_val = [-pi; pi/4; pi/4];
tau_max_val = 0.8;
tau_min_val = -0.8;