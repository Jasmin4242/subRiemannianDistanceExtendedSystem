classdef Trajectoryx < Trajectory
    properties (SetAccess = private)
        X
        stateNames
    end

    methods
        function obj = Trajectoryx(dT, X, stateNames, t0)
            if nargin < 4
                t0 = 0;
            end

            if ~isscalar(dT) || dT <= 0
                error("InputTrajectory:InvalidSamplingTime", ...
                    "dT must be a positive scalar.");
            end

            obj.dT = dT;
            obj.X = X;
            obj.stateNames = stateNames;
            obj.t0 = t0;
        end

        function N = length(obj)
            N = size(obj.X,1);
        end

        function nx = stateDimension(obj)
            nx = size(obj.X,2);
        end

        function saveTable(obj,skip)
            [~, git_hash] = system('git rev-parse --short HEAD');
            git_hash = strtrim(git_hash);
            data2pgftable(fullfile(dir_str,num2str(dT_des*1000),'X0123Y0123.txt'), data, 'git_hash',git_hash);
        end

    end
end