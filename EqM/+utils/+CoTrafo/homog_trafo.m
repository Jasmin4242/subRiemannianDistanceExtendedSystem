% homogeneous transformation matrix based on rotation matrix and translational vector 
function T = homog_trafo(S,r)
    T = [S, r;...
        zeros(1,3), 1];
end