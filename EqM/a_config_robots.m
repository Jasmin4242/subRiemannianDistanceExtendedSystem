% configuration scripts specifying numerical values of the employed robots

% chassis
% Diana_type = 'Diana_rund';
% if strcmp(Diana_type, 'Diana_rund')
M_val = 1.75; % chassis in kg
L_val = 0.24; % axis length in m
I_cz_val = 0.5*M_val*(0.29/2)^2; % chassis inertia around z-axis in kg m^2
% elseif strcmp(Diana_type, 'Diana_eckig')
%     M_val = 1.1; % chassis in kg
%     L_val = 0.18; % axis length in m
%     Steiner_Diana_eckig = M_val*0.09^2;
%     I_cz_val = 0.5*M_val*(0.29/2)^2 + Steiner_Diana_eckig; % chassis inertia around z-axis in kg m^2
% end
I_cx_val = nan;
I_cy_val = nan;

% wheel
m_val = 0.037; % wheel mass in kg
R_val = 0.035; % wheel radius in m
I_wx_val = m_val*(3*R_val^2 + 0.005^2)/12; % wheel inertia around x-axis in kg m^2
I_wy_val = 0.5*m_val*R_val^2;
I_wz_val = m_val*(3*R_val^2 + 0.005^2)/12; % wheel inertia around z-axis in kg m^2

% trailer
b1x_val = 1;%0.195; % hook length in m
Mt_val = 0.43; % trailer chassis in kg
Lt_val = L_val; % trailer axis length in m
I_tx_val = nan;
I_ty_val = nan;
I_tz_val = Mt_val*(0.1^2 + 0.16^2)/12; % chassis inertia around z-axis in kg m^2

% car
ell_val = 0.14; % distance between rear and front axle in m
Mcar_val = 1.05; % car mass in kg
I_carx_val = nan;
I_cary_val = nan;
I_carz_Steiner =  Mcar_val*0.08^2;
I_carz_val = Mcar_val*(0.23^2 + 0.15^2)/12 + I_carz_Steiner; % car inertia around z-axis in kg m
% (virtual) wheels for car
mcar_val = 2*0.05; % wheel mass in kg
Rcar_val = 0.03; % wheel radius in m
I_wcarx_val = 0.5*mcar_val*Rcar_val^2; % wheel inertia around x-axis in kg m^2
I_wcary_val = nan;
I_wcarz_val = mcar_val*(3*Rcar_val^2 + 0.01^2)/12; % wheel inertia around z-axis in kg m^2
% mcar_val = 0.04; % wheel mass in kg
% Rcar_val = 0.035; % wheel radius in m
% I_wcarx_val = 0.001; % wheel inertia around x-axis in kg m^2
% I_wcary_val = nan;
% I_wcarz_val = 0.005; % wheel inertia around z-axis in kg m^2

%%
robot.robot_type = robot_type;
robot.R = R_val;
robot.L = L_val;
robot.Lt = Lt_val;
robot.b1x = b1x_val;
% car
robot.ell = ell_val;

save('results/config.mat')