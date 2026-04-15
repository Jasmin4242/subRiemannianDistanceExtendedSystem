classdef Robot
    properties
        Name
    end

    properties (SetAccess = private)
        AvailableModels
        ActiveModelName
        axisLenght
    end

    methods
        function obj = Robot(name)
            obj.Name = name;

            % Map für Modelle
            obj.AvailableModels = containers.Map('KeyType', 'char', 'ValueType', 'any');
        
            % Modelle hinzufügen
            obj = obj.initializeModels();
        
            % Standardmodell setzen
            modelNames = keys(obj.AvailableModels);
            obj.ActiveModelName = modelNames{1};

            % axis length for visualization
            data_config = load('EqM/results/config.mat');
            obj.axisLenght = data_config.L_val;
        end

        function obj = initializeModels(obj)
            data = load('EqM/results/EqM_Diana.mat');
            
            %% Kinematics
            stateNames = ["x" "y" "theta" "phi_l"];
            inputNames = ["v" "omega"];
            f = @(y,u) data.G_y_func_v_omega(y) * u; %x_dot = f(x,u)            
            uMax = [0.2; pi/2];
            uMin = -uMax;
            nx = 4;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 0];
            costParams.q_z = [1 0.1 5 0]';
            costParams.q_u = [0.125 0.0125];
            costParams.r = [1 1 2 1];   
            costParams.d = 2*prod(costParams.r);
            kinematics_vw = Model('kinematics_vw', f, constraints, costParams,nx,nu, stateNames, inputNames);
            
            stateNames = ["x" "y" "theta" "phi_l"];
            inputNames = ["phi_dot_l" "phi_dot_r"];
            f = @(y,u) data.G_y_func(y) * u; %x_dot = f(x,u)            
            uMax = 10*ones(2,1);
            uMin = -uMax;
            nx = 4;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 0];
            costParams.q_z = [1 0.1 5 0]';
            costParams.q_u = 2e-5*ones(2,1);
            costParams.r = [1 1 2 1];
            costParams.d = 2*prod(costParams.r);
            kinematics_phidot = Model('kinematics_phidot', f, constraints, costParams,nx,nu, stateNames, inputNames);

            %% Dynamics
            stateNames = ["x" "y" "theta" "phi_l" "phi_l_dot" "phi_r_dot"];
            inputNames = ["phi_dot_l" "phi_dot_r"];
            f_kin = @(y,u) data.G_y_func(y) * [y(5);y(6)];            
            f = @(y,u) [f_kin(y,u);data.Minv_eqM_func(y)*u];
            uMax = 0.2*ones(2,1);
            uMin = -uMax;
            nx = 6;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 1 0 0 0 0; zeros(1,6); 0 0 0 0 1 0; 0 0 0 0 0 1];
            costParams.q_z = [1 0.1 5 0 2e-5*ones(1,2)]';
            costParams.q_u = 1*ones(2,1);
            costParams.r = [1 1 2 1 1 1];
            costParams.d = 2*prod(costParams.r);
            dynamics_phidot = Model('dynamics_phidot', f, constraints, costParams,nx,nu, stateNames, inputNames);

       
            % available models
            obj.AvailableModels('kinematics_vw') = kinematics_vw;
            obj.AvailableModels('kinematics_phidot') = kinematics_phidot;
            obj.AvailableModels('dynamics_phidot') = dynamics_phidot;
        end

        function obj = setModel(obj, name)   
            name = char(name);
            if ~isKey(obj.AvailableModels, name)
                error("Robot:UnknownModel", ...
                    "Model '%s' not available.", name);
            end        
            obj.ActiveModelName = name;
        end

        function config = getModel(obj)
            config = obj.AvailableModels(char(obj.ActiveModelName));
        end

    end
end