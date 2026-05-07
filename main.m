clear all;
close all;
addpath(genpath(pwd));

%% user params
dT = 0.25;

save_results = 0;
store_git_info = 0;
save_video = 0;

% choose the vehicle
vehicle = ArticulatedVehicleCollapsed('artVeh');
% vehicle = ArticulatedVehicle('artVeh');
% vehicle = Trailer('trailer');
% vehicle = Robot('DIANA');

% choose the model
% modelname = 'kinematics_vw';
% modelname = 'kinematics_phidot';
modelname = 'dynamics_phidot';
% modelname = 'dynamics_vw';


% choose the initial condition
if isa(vehicle,'Robot')
    if startsWith(modelname, 'dynamics')
        x0 = [-0.6; -0.75; 0 ;0; 0; 0];
    else
        x0 = [-0.6; -0.75; 0 ;0];
    end
elseif isa(vehicle,'Trailer')
    if startsWith(modelname, 'dynamics')
        x0 = [-0.6; -0.75; 0; 0; 0; 0; 0; 0];
    else
        x0 = [-0.6; -0.75; 0; 0; 0; 0];
    end    
elseif isa(vehicle,'ArticulatedVehicle')
    if startsWith(modelname, 'dynamics')
        %x y theta gamma phi phi_t phi1_dot phi2_dot ODER
        %x y theta gamma phi phi_t v omega 
        x0 = [-0.6; -0.75; 0; 0; 0; 0; 0.5; 0.5];
    else
        x0 = [-0.6; -0.75; 0; 0; 0; 0];
    end    
elseif isa(vehicle,'ArticulatedVehicleCollapsed')
    if startsWith(modelname, 'dynamics')
        %x y theta gamma phi phi_t phi1_dot gama_dot ODER
        %x y theta gamma phi phi_t v gama_dot 
        x0 = [-0.6; -0.75; 0; 0; 0; 0; 0.5; 0];
    else
        x0 = [-0.6; -0.75; 0; 0; 0; 0];
    end    
else
    warning('vehicle unknown')
end

%% initialization
% Casadi
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


%% open-loop simulation
% U_data = repmat([0.00003 0], 100, 1);
U_data = repmat([0 0.00005], 100, 1); %Gelenk wird aktuiert in positive Richtung
% U_data = repmat([-0.00003 -0.0001], 100, 1); %gegensätzliche Bewegung
u = Trajectoryu(dT, U_data,vehicle.getModel().inputNames, 0);
x = simulator.simulate(x0, u);


% %% MPC
% horizon = 60;
% mpcController = MPCController(vehicle, dT, horizon);
% 
% k_sim = 300;
% [x,u,cost] = simulator.sim_MPC_closedLoop(x0, k_sim,mpcController);

%% postprocessing
% VISUAlIZER
visualizer = Visualizer();
visualizer.animation(x, vehicle, save_video);
visualizer.plotStates(x);
visualizer.plotInputs(u);
% visualizer.plotPath(x);
% visualizer.plotCosts(cost);  

%% store results
if save_results == 1    
    skip = 1;
    x.saveTable(skip,store_git_info);
    u.saveTable(skip,store_git_info);
    cost.saveTable(skip,store_git_info);
end

