clear all;
close all;

addpath(genpath(pwd));

%for Casadi
addpath(genpath('/scratch/tmp/jkrauspenhaar/temp/Matlabinstallationen/casadi-3.6.7-linux64-matlab2018b'));

dT = 0.05;
robot = Robot('DIANA');
simulator = Simulator(robot.Kinematics, dT);

%% MPC
horizon = 30;
constraints = ConstraintSet(robot);
mpcController = MPCController(robot, robot.Kinematics, dT, horizon, constraints);
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