classdef Trajectoryu < Trajectory
    properties (SetAccess = private)
        U
    end

    methods
        function obj = Trajectoryu(dT, U, t0)
            if nargin < 3
                t0 = 0;
            end

            if ~isscalar(dT) || dT <= 0
                error("InputTrajectory:InvalidSamplingTime", ...
                    "dT must be a positive scalar.");
            end

            obj.dT = dT;
            obj.U = U;
            obj.t0 = t0;
        end

        function u = getInputk(obj, k)
            if k < 1 || k > size(obj.U,1)
                error("InputTrajectory:IndexOutOfRange", ...
                    "Input index k is out of range.");
            end
            u = obj.U(k,:).';
        end


        function u = getInput(obj)
            u = obj.U;
        end

        function N = length(obj)
            N = size(obj.U,1);
        end


        function nu = inputDimension(obj)
            nu = size(obj.U,2);
        end


    end
end