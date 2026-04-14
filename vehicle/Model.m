classdef Model
    properties (SetAccess = private)
        Name
        f           % struct mit f(x,u)
        Constraints     % ConstraintSet Objekt
        CostParameters  % struct oder später eigene Klasse
        nx
        nu
    end

    methods
        function obj = Model(name, f, constraints, costParams, nx, nu)

            obj.Name = string(name);
            obj.f = f;
            obj.Constraints = constraints;
            obj.CostParameters = costParams;
            obj.nx = nx;
            obj.nu =nu;
        end
    end
end