classdef Simulator
    properties (SetAccess = private)
        Model
        dT
        Method
    end

    methods
        function obj = Simulator(model, dT, method)
            if nargin < 3
                method = "ode45";
            end

            if ~isprop(model, 'f')
                error("Simulator:InvalidModel", ...
                    "Model must contain a field f with xdot = f(x,u).");
            end

            obj.Model = model;
            obj.dT = dT;
            obj.Method = string(method);
        end

        function [x,u,cost] = sim_MPC_closedLoop(obj,xcurrent, k_sim,mpcController)
            y_init = mpcController.initialGuessFirst(xcurrent);
            x_data = zeros(k_sim,4);
            x_data(1,:) = xcurrent;
            u_data = zeros(k_sim,2);
            cost_data =zeros(k_sim,1);
            for kk = 1:k_sim
                kk
                %compute input
                [u0, sol] = mpcController.computeInput(xcurrent, y_init);
                u_data(kk,:) = u0;
                y_init = mpcController.initialGuess(sol);
                %apply input
                xcurrent = obj.simstep(xcurrent,u0);
                x_data(kk,:) = xcurrent;
                cost_data(kk) = sol.Jval;
            end
            x = Trajectoryx(obj.dT, x_data);
            u = Trajectoryu(obj.dT, u_data);
            cost = Trajectorycost(obj.dT,cost_data);
        end

        function outputTrajectory = simulate(obj, x0, inputTrajectory)
            if abs(inputTrajectory.dT - obj.dT) > 1e-12
                error("Simulator:SamplingTimeMismatch", ...
                    "Sampling time of simulator and input trajectory must match.");
            end

            N = inputTrajectory.length();
            nx = numel(x0);
            X = zeros(N+1, nx);

            X(1,:) = x0(:).';

            xk = x0(:);

            for k = 1:N
                uk = inputTrajectory.getInputk(k);
                xk = obj.simstep(xk, uk);
                X(k+1,:) = xk.';
            end

            outputTrajectory = Trajectoryx(obj.dT,X);
        end

        function xNext = simstep(obj, x, u)
            f = obj.Model.f;
            dT = obj.dT;

            switch lower(obj.Method)
                case "euler"
                    xNext = x + dT * f(x,u);

                case "rk4"
                    k1 = f(x, u);
                    k2 = f(x + 0.5*dT*k1, u);
                    k3 = f(x + 0.5*dT*k2, u);
                    k4 = f(x + dT*k3, u);

                    xNext = x + (dT/6) * (k1 + 2*k2 + 2*k3 + k4);

                case "ode45"                
                    [~, xtraj] = ode45(@(t, x) f(x, u), [0 dT], x);                
                    xNext = xtraj(end, :).';

                otherwise
                    error("Simulator:UnknownMethod", ...
                        "Unknown simulation method: %s", obj.Method);
            end
        end
        
    end

    % methods (Access = private)
    % 
    % end
end