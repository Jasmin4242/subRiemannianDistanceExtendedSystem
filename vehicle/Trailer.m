classdef Trailer
    properties
        Name
    end

    properties (SetAccess = private)
        AvailableModels
        ActiveModelName
        axisLength
        axisLengthTrailer
        hookLength
    end

    methods
        function obj = Trailer(name)
            obj.Name = name;

            % Map für Modelle
            obj.AvailableModels = containers.Map('KeyType', 'char', 'ValueType', 'any');
        
            % Modelle hinzufügen
            obj = obj.initializeModels();
        
            % Standardmodell setzen
            modelNames = keys(obj.AvailableModels);
            obj.ActiveModelName = modelNames{1};
            
            data_config = load('EqM/results/config.mat');
            obj.axisLength = data_config.L_val;
            obj.axisLengthTrailer = data_config.Lt_val;
            obj.hookLength = data_config.b1x_val;
        end

        function obj = initializeModels(obj)
            data = load('EqM/results/EqM_trailer.mat');
            data_config = load('EqM/results/config.mat');
            b1x = data_config.b1x_val;
            
            %% Kinematics
            stateNames = ["x" "y" "theta" "theta_t" "phi_l" "phi_1_t"];
            inputNames = ["v" "omega"];
            f = @(y,u) data.G_y_func_v_omega(y) * u; %x_dot = f(x,u)            
            uMax = [0.4; pi/8];
            uMin = -uMax;
            nx = 6;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0 0 0;... %x1=z1
                            0 0 1 0 0 0;... %x3=z2
                            0 1 0 0 0 0;... %x2=z3
                            0 b1x 0 -b1x^2 0 0;...%x2-x4=z4
                            zeros(1,6);...
                            zeros(1,6)];          
            costParams.q_z = [0.1 0.1 5 5 0 0]';
            costParams.q_u = [0.125 0.0125];
            costParams.r = [1 1 2 3 1 1];  
            costParams.d = prod(costParams.r);
            kinematics_vw = Model('kinematics_vw', f, constraints, costParams,nx,nu, stateNames, inputNames);
            
            stateNames = ["x" "y" "theta" "theta_t" "phi_l" "phi_1_t"];
            inputNames = ["phi_dot_l" "phi_dot_r"];
            f = @(y,u) data.G_y_func(y) * u; %x_dot = f(x,u)            
            uMax = 10*ones(2,1);
            uMin = -uMax;
            nx = 6;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0 0 0;... %x1=z1
                            0 0 1 0 0 0;... %x3=z2
                            0 1 0 0 0 0;... %x2=z3
                            0 b1x 0 -b1x^2 0 0;...%x2-x4=z4
                            zeros(1,6);...
                            zeros(1,6)];          
            costParams.q_z = [0.1 0.1 5 5 0 0]';
            costParams.q_u = 2e-5*ones(2,1);
            costParams.r = [1 1 2 3 1 1];  
            costParams.d = prod(costParams.r);
            kinematics_phidot = Model('kinematics_phidot', f, constraints, costParams,nx,nu, stateNames, inputNames);

            %% Dynamics
            stateNames = ["x" "y" "theta" "theta_t" "phi_l" "phi_1_t" "phi_l_dot" "phi_r_dot"];
            inputNames = ["tau_l" "tau_r"];
            f_kin = @(y,u) data.G_y_func(y) * [y(7);y(8)];            
            f = @(y,u) [f_kin(y,u);data.Minv_eqM_func(y)*u];
            uMax = 0.02*ones(2,1);
            uMin = -uMax;
            nx = 8;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0 0 0 0 0;...
                            0 0 1 0 0 0 0 0;...
                            0 1 0 0 0 0 0 0;...
                            0 b1x 0 -b1x^2 0 0 0 0;...
                            zeros(1,8);...
                            zeros(1,8);...
                            0 0 0 0 0 0 1 0;...
                            0 0 0 0 0 0 0 1];
            costParams.q_z = [0.1 0.1 5 5 0 0 2e-5*ones(1,2)]';
            costParams.q_u = 1*ones(2,1);
            costParams.r = [1 1 2 3 1 1 1 1];
            costParams.d = prod(costParams.r);
            dynamics_phidot = Model('dynamics_phidot', f, constraints, costParams,nx,nu, stateNames, inputNames);

            stateNames = ["x" "y" "theta" "theta_t" "phi_l" "phi_1_t" "v" "w"];
            inputNames = ["tau_l" "tau_r"];
            f_kin = @(y,u) data.G_y_func_v_omega(y) * [y(7);y(8)];            
            f = @(y,u) [f_kin(y,u);data.Minv_eqM_func_vw(y)*u];
            uMax = 0.005*ones(2,1);
            uMin = -uMax;
            nx = 8;
            nu = 2;
            constraints = ConstraintSet(obj, nx, nu, uMin, uMax);
            costParams.z = [1 0 0 0 0 0 0 0;...
                            0 0 1 0 0 0 0 0;...
                            0 1 0 0 0 0 0 0;...
                            0 b1x 0 -b1x^2 0 0 0 0;...
                            zeros(1,8);...
                            zeros(1,8);...
                            0 0 0 0 0 0 1 0;...
                            0 0 0 0 0 0 0 1];
            costParams.q_z = [0.1 0.1 5 5 0 0 0.125*ones(1,2)]';
            costParams.q_u = 1*ones(2,1);
            costParams.r = [1 1 2 3 1 1 1 1];
            costParams.d = prod(costParams.r);
            dynamics_vw = Model('dynamics_vw', f, constraints, costParams,nx,nu, stateNames, inputNames);
       
            % available models
            obj.AvailableModels('kinematics_vw') = kinematics_vw;
            obj.AvailableModels('kinematics_phidot') = kinematics_phidot;
            obj.AvailableModels('dynamics_phidot') = dynamics_phidot;
            obj.AvailableModels('dynamics_vw') = dynamics_vw;
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