classdef Simulator
    properties (SetAccess = private)
        Model
        Ts
        Method
    end

    methods
        function obj = Simulator(model, Ts, method)
            if nargin < 3
                method = "rk4";
            end

            if ~isfield(model, 'f')
                error("Simulator:InvalidModel", ...
                    "Model must contain a field f with xdot = f(x,u).");
            end

            if ~isscalar(Ts) || Ts <= 0
                error("Simulator:InvalidSamplingTime", ...
                    "Ts must be a positive scalar.");
            end

            obj.Model = model;
            obj.Ts = Ts;
            obj.Method = string(method);
        end

        function result = simulate(obj, x0, inputTrajectory)
            if abs(inputTrajectory.Ts - obj.Ts) > 1e-12
                error("Simulator:SamplingTimeMismatch", ...
                    "Sampling time of simulator and input trajectory must match.");
            end

            N = inputTrajectory.length();
            nx = numel(x0);
            nu = inputTrajectory.inputDimension();

            X = zeros(N+1, nx);
            U = zeros(N, nu);
            T = inputTrajectory.t0 + (0:N)' * obj.Ts;

            X(1,:) = x0(:).';

            xk = x0(:);

            for k = 1:N
                uk = inputTrajectory.getInput(k);
                U(k,:) = uk.';
                xk = obj.step(xk, uk);
                X(k+1,:) = xk.';
            end

            result = struct();
            result.T = T;
            result.X = X;
            result.U = U;
            result.Ts = obj.Ts;
            result.method = obj.Method;
        end
    end

    methods (Access = private)
        function xNext = step(obj, x, u)
            f = obj.Model.f;
            Ts = obj.Ts;

            switch lower(obj.Method)
                case "euler"
                    xNext = x + Ts * f(x,u);

                case "rk4"
                    k1 = f(x, u);
                    k2 = f(x + 0.5*Ts*k1, u);
                    k3 = f(x + 0.5*Ts*k2, u);
                    k4 = f(x + Ts*k3, u);

                    xNext = x + (Ts/6) * (k1 + 2*k2 + 2*k3 + k4);

                otherwise
                    error("Simulator:UnknownMethod", ...
                        "Unknown simulation method: %s", obj.Method);
            end
        end
    end
end