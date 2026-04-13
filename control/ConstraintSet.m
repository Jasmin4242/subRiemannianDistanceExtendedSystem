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
        function obj = ConstraintSet(vehicle, uMin, uMax, duMin, duMax, xMin, xMax)
            if nargin < 4, duMin = -1*ones(vehicle.inputDimension(),1); end %not used so far!
            if nargin < 5, duMax = 1*ones(vehicle.inputDimension(),1); end %not used so far!
            if nargin < 6, xMin = -inf*ones(vehicle.stateDimension(),1); end
            if nargin < 7, xMax = inf*ones(vehicle.stateDimension(),1); end

            obj.xMin = xMin;
            obj.xMax = xMax;
            obj.uMin = uMin;
            obj.uMax = uMax;
            obj.duMin = duMin; %not used so far!
            obj.duMax = duMax; %not used so far!
        end
    end
end