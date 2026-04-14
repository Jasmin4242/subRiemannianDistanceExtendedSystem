clear all;
close all;

%% user params
dT = 0.25;
trailer = Trailer('trailer');

% Modell auswählen
trailer = trailer.setModel('kinematics_vw');
% trailer = trailer.setModel('kinematics_phidot');
% x0 = [-0.6; -0.75; 0; 0; 0; 0];
x0 = [0; 0.4; 5*pi/180; 5*pi/180; 0; 0];

% trailer = trailer.setModel('dynamics_phidot');
% x0 = [-0.6; -0.75; 0 ;0; 0; 0];

%% initialization
addpath(genpath(pwd));

%for Casadi
addpath(genpath('/scratch/tmp/jkrauspenhaar/temp/Matlabinstallationen/casadi-3.6.7-linux64-matlab2018b'));

simulator = Simulator(trailer.getModel(), dT);

%% MPC
horizon = 60;
mpcController = MPCController(trailer, dT, horizon);

k_sim = 100;
[x,u,cost] = simulator.sim_MPC_closedLoop(x0, k_sim,mpcController);

%% postprocessing
% VISUAlIZER
visualizer = Visualizer();
visualizer.plotStates(x);
visualizer.plotInputs(u);
visualizer.plotPath(x);
visualizer.plotCosts(cost);

disp('Endzustand')
disp('x und y')
x.X(end,1:2)
disp('theta und theta_t')
x.X(end,3:4)*180/pi


% figure();
% plot(abs(x.X(:,3)-x.X(:,4))*180/pi)