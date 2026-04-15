classdef Trajectoryu < Trajectory
    properties (SetAccess = private)
        U
        inputNames
    end

    methods
        function obj = Trajectoryu(dT, U,inputNames, t0)
            if nargin < 4
                t0 = 0;
            end

            if ~isscalar(dT) || dT <= 0
                error("InputTrajectory:InvalidSamplingTime", ...
                    "dT must be a positive scalar.");
            end

            obj.dT = dT;
            obj.U = U;
            obj.inputNames = inputNames;
            obj.t0 = t0;
        end

        function N = length(obj)
            N = size(obj.U,1);
        end


        function nu = inputDimension(obj)
            nu = size(obj.U,2);
        end
    end
end