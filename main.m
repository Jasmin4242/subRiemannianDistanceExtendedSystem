clear all;
close all;

%% user params
dT = 0.25;

% choose the vehicle
% vehicle = Trailer('trailer');
vehicle = Robot('DIANA');

% choose the model
% modelname = 'kinematics_vw';
% modelname = 'kinematics_phidot';
modelname = 'dynamics_phidot';

% git and storing results
store_git_info = false;
save_results = true;

% choose the initial condition
if isa(vehicle,'Robot')
    if startsWith(modelname, 'dynamics')
        x0 = [-0.6; -0.75; 0 ;0; 0; 0];
    else
        x0 = [-0.6; -0.75; 0 ;0];
    end
elseif isa(vehicle,'Trailer')
    if startsWith(modelname, 'dynamics')
        x0 = [0; 0.4; 5*pi/180; 5*pi/180; 0; 0; 0; 0];
    else
        x0 = [-0.6; -0.75; 0; 0; 0; 0];
    end    
else
    warning('vehicle unknown')
end

%% initialization
% add paths
addpath(genpath(pwd));
addpath(genpath('/scratch/tmp/jkrauspenhaar/temp/Matlabinstallationen/casadi-3.6.7-linux64-matlab2018b'));

%git
if save_results && store_git_info
    status_flag = system('git add . && git diff --quiet && git diff --cached --quiet');
    if status_flag ~= 0
        error('There are uncommitted changes or untracked files. Please commit everything before producing results.')
    end
end

vehicle = vehicle.setModel(modelname);
simulator = Simulator(vehicle.getModel(), dT);

%% MPC
horizon = 60;
mpcController = MPCController(vehicle, dT, horizon);

k_sim = 2; %200;
[x,u,cost] = simulator.sim_MPC_closedLoop(x0, k_sim,mpcController);

%% postprocessing
% VISUAlIZER
visualizer = Visualizer();
visualizer.animation(x, vehicle);
visualizer.plotStates(x);
visualizer.plotInputs(u);
visualizer.plotPath(x);
visualizer.plotCosts(cost);  

