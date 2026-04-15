classdef Trajectorycost < Trajectory
    properties (SetAccess = private)
        cost
    end

    methods
        function obj = Trajectorycost(dT, cost_data, t0)
            if nargin < 3
                t0 = 0;
            end

            if ~isscalar(dT) || dT <= 0
                error("InputTrajectory:InvalidSamplingTime", ...
                    "dT must be a positive scalar.");
            end

            obj.dT = dT;
            obj.cost = cost_data;
            obj.t0 = t0;
        end

        function c = getCostk(obj, k)
            if k < 1 || k > size(obj.X,1)
                error("InputTrajectory:IndexOutOfRange", ...
                    "Input index k is out of range.");
            end
            c = obj.cost(k,:).';
        end


        function c = getCost(obj)
            c = obj.cost;
        end

        function N = length(obj)
            N = size(obj.cost,1);
        end


        function saveTable(obj,skip, store_git_info)
            data.time = obj.timeGrid();
            data.costs = obj.cost;            

            if store_git_info == false
                warning('no git info stored')
                data2pgftable('results/Trajectorycosts.txt', data,'skip',skip);
            else
                [~, git_hash] = system('git rev-parse --short HEAD');
                git_hash = strtrim(git_hash);
                data2pgftable('results/Trajectorycosts.txt', data, 'git_hash',git_hash,'skip',skip);
            end
        end

    end
end