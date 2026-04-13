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
            if nargin < 6, xMin = -inf*ones(vehicle.stateDimension(),1); end
            if nargin < 7, xMax = inf*ones(vehicle.stateDimension(),1); end

            % %v w as inputs
            % if nargin < 4, uMin = [-0.2; -pi/2]; end
            % if nargin < 5, uMax = [0.2; pi/2]; end
            % if nargin < 6, duMin = uMin/4; end
            % if nargin < 7, duMax = uMax/4; end
            
            % %wheel speed as inputs
            % if nargin < 4, uMin = [-1; -1]; end
            % if nargin < 5, uMax = [1; 1]; end
            % if nargin < 6, duMin = uMin/4; end
            % if nargin < 7, duMax = uMax/4; end

            obj.xMin = xMin;
            obj.xMax = xMax;
            obj.uMin = uMin;
            obj.uMax = uMax;
            obj.duMin = duMin;
            obj.duMax = duMax;
        end
    end
end