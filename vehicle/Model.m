classdef Model
    properties (SetAccess = private)
        Name
        f           % struct mit f(x,u)
        Constraints     % ConstraintSet Objekt
        CostParameters  % struct oder später eigene Klasse
    end

    methods
        function obj = Model(name, f, constraints, costParams)
            if nargin < 4
                error("ModelConfig:InvalidInput", ...
                    "Name, Model, Constraints, CostParameters required.");
            end

            obj.Name = string(name);
            obj.f = f;
            obj.Constraints = constraints;
            obj.CostParameters = costParams;
        end
    end
end