% perform Roessselsprung
function omega = roessel(omega_tilde)
    omega = [omega_tilde(3,2); omega_tilde(1,3); omega_tilde(2,1)];
end