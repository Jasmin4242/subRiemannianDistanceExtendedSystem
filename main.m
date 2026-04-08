clear all;
close all;

addpath(genpath(pwd));

dT = 0.05;

% DIANA
robot = Robot('DIANA');
%functions
kin = robot.getKinematics();
dyn = robot.getDynamics();
nx = robot.stateDimension();
nu = robot.inputDimension();


% INPUT TRAJECTORY
u = Trajectoryu(dT, [0.1*ones(10,1) 0.2*ones(10,1)]);
%functions
u_1 = u.getInputk(1);
u_vals = u.getInput();
N = u.length();
t_1 = u.getTime(1);
nu = u.inputDimension();
t_grid = u.timeGrid();

% STATE TRAJECTORY
x = Trajectoryx(dT, [0*ones(10,1) 0.1*ones(10,1) 0.2*ones(10,1)]);
%functions
x_1 = x.getStatek(1);
x_vals = x.getState();
N = x.length();
t_1 = x.getTime(1);
nx = x.stateDimension();
t_grid = x.timeGrid();

% SIMULATOR
% to do
   