classdef Model
    properties (SetAccess = private)
        Name
        f          
        Constraints     
        CostParameters  
        nx
        nu
        stateNames
        inputNames
    end

    methods
        function obj = Model(name, f, constraints, costParams, nx, nu, stateNames, inputNames)

            obj.Name = string(name);
            obj.f = f;
            obj.Constraints = constraints;
            obj.CostParameters = costParams;
            obj.nx = nx;
            obj.nu =nu;
            obj.stateNames = stateNames;
            obj.inputNames = inputNames;
        end
    end
end