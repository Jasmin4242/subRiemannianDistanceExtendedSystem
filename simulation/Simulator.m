classdef Simulator
    properties (SetAccess = private)
        Model
        dT
        Method
    end

    methods
        function obj = Simulator(model, dT, method)
            if nargin < 3
                method = "rk4";
            end

            if ~isfield(model, 'f')
                error("Simulator:InvalidModel", ...
                    "Model must contain a field f with xdot = f(x,u).");
            end

            obj.Model = model;
            obj.dT = dT;
            obj.Method = string(method);
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