clear all;
close all;

%% user params
dT = 0.05;
robot = Robot('DIANA');
% Modell auswählen
% robot = robot.setModel("kinematics_vw");
robot = robot.setModel("kinematics_phidot");


%% initialization
addpath(genpath(pwd));

%for Casadi
addpath(genpath('/scratch/tmp/jkrauspenhaar/temp/Matlabinstallationen/casadi-3.6.7-linux64-matlab2018b'));

simulator = Simulator(robot.getModel(), dT);

%% MPC
horizon = 30;
mpcController = MPCController(robot, dT, horizon);
x0 = [-0.6; -0.75; 0 ;0];

k_sim = 300;
[x,u,cost] = simulator.sim_MPC_closedLoop(x0, k_sim,mpcController);

%% postprocessing
% VISUAlIZER
visualizer = Visualizer();
visualizer.plotStates(x);
visualizer.plotInputs(u);
visualizer.plotPath(x);
visualizer.plotCosts(cost);