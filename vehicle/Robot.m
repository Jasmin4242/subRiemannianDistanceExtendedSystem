classdef Robot
    properties
        Name
    end

    properties (SetAccess = private)
        AvailableModels
        ActiveModelName
        nx
        nu
        state_names
        input_names
    end

    methods
        function obj = Robot(name)
            obj.Name = name;

            obj.nx = 4;
            obj.nu = 2;

            % Map für Modelle
            obj.AvailableModels = containers.Map();
        
            % Modelle hinzufügen
            obj = obj.initializeModels();
        
            % Standardmodell setzen
            modelNames = obj.AvailableModels.keys();
            obj.ActiveModelName = modelNames{1};
        end

        function obj = initializeModels(obj)
            data = load('EqM/results/EqM_Diana.mat');

            % kinematic with velocities v and w as inputs
            f = @(y,u) data.G_y_func_v_omega(y) * u; %x_dot = f(x,u)            
            uMax = [0.2; pi/2];
            uMin = -uMax;
            constraints = ConstraintSet(obj, uMin, uMax);
            costParams.z = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 0];
            costParams.q_x = [1 5 0.1 0]';
            costParams.q_u = [0.125 0.0125];
            costParams.r = [1 1 2 1];
            kinematics_vw = Model("kinematics_vw", f, constraints, costParams);

            % kinematic with wheel velocities phi_dot left and right
            f = @(y,u) data.G_y_func(y) * u; %x_dot = f(x,u)            
            uMax = 10*ones(2,1);
            uMin = -uMax;
            constraints = ConstraintSet(obj, uMin, uMax);
            costParams.z = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 0];
            costParams.q_x = [1 5 0.1 0]';
            costParams.q_u = 2e-5*ones(2,1);
            costParams.r = [1 1 2 1];
            kinematics_phidot = Model("kinematics_phidot", f, constraints, costParams);
       
            % available models
            obj.AvailableModels("kinematics_vw") = kinematics_vw;
            obj.AvailableModels("kinematics_phidot") = kinematics_phidot;
        end

        function obj = setModel(obj, name)   
            name = string(name);
            if ~isKey(obj.AvailableModels, name)
                error("Robot:UnknownModel", ...
                    "Model '%s' not available.", name);
            end        
            obj.ActiveModelName = name;
        end

        function config = getModel(obj)
            config = obj.AvailableModels(obj.ActiveModelName);
        end

        function nx = stateDimension(obj)
            nx = obj.nx;
        end

        function nu = inputDimension(obj)
            nu = obj.nu;
        end

    end
end