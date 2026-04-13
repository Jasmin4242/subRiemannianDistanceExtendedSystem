classdef ConstraintSet
    properties (SetAccess = private)
        xMin
        xMax
        uMin
        uMax
        duMin
        duMax
    end

    methods
        function obj = ConstraintSet(vehicle, xMin, xMax, uMin, uMax, duMin, duMax)
            if nargin < 2, xMin = -inf*ones(vehicle.stateDimension(),1); end
            if nargin < 3, xMax = inf*ones(vehicle.stateDimension(),1); end
            if nargin < 4, uMin = [-10; -10]; end
            if nargin < 5, uMax = [10; 10]; end
            if nargin < 6, duMin = [-5; -5]; end
            if nargin < 7, duMax = [5; 5]; end

            obj.xMin = xMin;
            obj.xMax = xMax;
            obj.uMin = uMin;
            obj.uMax = uMax;
            obj.duMin = duMin;
            obj.duMax = duMax;
        end
    end
end