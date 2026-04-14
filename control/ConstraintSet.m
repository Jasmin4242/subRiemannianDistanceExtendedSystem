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
        function obj = ConstraintSet(vehicle, nx, nu, uMin, uMax, duMin, duMax, xMin, xMax)
            if nargin < 6, duMin = -1*ones(nu,1); end %not used so far!
            if nargin < 7, duMax = 1*ones(nu,1); end %not used so far!
            if nargin < 8, xMin = -inf*ones(nx,1); end
            if nargin < 9, xMax = inf*ones(nx,1); end

            obj.xMin = xMin;
            obj.xMax = xMax;
            obj.uMin = uMin;
            obj.uMax = uMax;
            obj.duMin = duMin; %not used so far!
            obj.duMax = duMax; %not used so far!
        end
    end
end