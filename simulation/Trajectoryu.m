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

        function saveTable(obj,skip, store_git_info)
            data.time = obj.timeGrid();
            for j = 1:length(obj.inputNames)
                data.(obj.inputNames(j)) = obj.U(:,j);
            end

            if store_git_info == false
                warning('no git info stored')
                data2pgftable('results/Trajectoryu.txt', data,'skip',skip);
            else
                [~, git_hash] = system('git rev-parse --short HEAD');
                git_hash = strtrim(git_hash);
                data2pgftable('results/Trajectoryu.txt', data, 'git_hash',git_hash,'skip',skip);
            end
        end

    end
end