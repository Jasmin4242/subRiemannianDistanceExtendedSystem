classdef Robot
    properties
        Name
    end

    properties (SetAccess = private)
        Kinematics
        Dynamics
        nx
        nu
        state_names
        input_names
    end

    methods
        function obj = Robot(name)
            obj.Name = name;
            data = load('EqM/results/EqM_Diana.mat');
            %Kinematics continous time!
            obj.Kinematics.G = data.G_y_func; %4x2, state [x y theta phi_1], input [ph1_1 phi_2]
            obj.Kinematics.f = @(y,u) data.G_y_func_v_omega(y) * u; %x_dot = f(x,u)
            %Dynamics continous time
            obj.Dynamics.M = data.M_eqM_func;
            obj.Dynamics.Minv = data.Minv_eqM_func;
            obj.Dynamics.k = data.k_eqM_func;
            obj.Dynamics.q = data.q_eqM_func;
            obj.nx = 4;
            obj.nu = 2;
        end

        function kin = getKinematics(obj)
            kin = obj.Kinematics;
        end

        function dyn = getDynamics(obj)
            dyn = obj.Kinematics;
        end

        function nx = stateDimension(obj)
            nx = obj.nx;
        end

        function nu = inputDimension(obj)
            nu = obj.nu;
        end

    end
end