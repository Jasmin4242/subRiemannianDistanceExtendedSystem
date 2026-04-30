% script to derive the equations of motion of robot-trailer system 
% for simulation purposes of the plant 
% mr, 3.11.2025


function EqM_articulatedVehicle_kin_vOmega

% generalized coordinates
syms xc(t) yc(t) theta(t) phi1(t) phi2(t) xt(t) yt(t) phi1t(t) phi2t(t) gama(t) % note that here plane=inertial frame (no tilted plane)
assume([xc(t), yc(t), theta(t), phi1(t), phi2(t), ...
    xt(t), yt(t), phi1t(t), phi2t(t), gama(t)], 'real')
y = [xc; yc; theta; gama; phi1; phi1t];
assume(y(t), 'real');
Dy = diff(y,t);
% generalized velocities
syms Dphi1(t) Dphi2(t)
s = [Dphi1; Dphi2];
assume(s(t), 'real');
% geometrics
syms h L Lt R l l_t real % real
% Lt = L;
% masses
syms M Mt m real
% inertia
syms I_cx I_cy I_cz real;
syms I_tx I_ty I_tz real;
syms I_wx I_wy I_wz real;
% applied forces and torques
syms g tau_w1 tau_w2 real
% u = [tau_w1; tau_w2];

%% Constraint on phi2 and phi2t
phi2_0 = 0; % initial condition (may implement w.r.t. theta0)
phi2 = phi1 + L*theta/R + phi2_0;
phi2t_0 = 0; % initial condition (may implement w.r.t. theta0)
phi2t = phi1t + Lt*(theta-gama)/R + phi2t_0;

%% Position vectors and transformations
% position vectors in robot (chassis frame) and wheel frame
r_c_C = [0;0;0]; % chassis vector in chassis COS
r_ct_T = [0;0;0]; % trailer chassis vector in trailer COS
r_w1_W1 = [0;0;0]; % wheel 1 vector in wheel 1 COS
r_w2_W2 = [0;0;0]; % wheel 2 vector in wheel 2 COS
r_tw1_tW1 = [0;0;0]; % trailer wheel 1 vector in trailer wheel 1 COS
r_tw2_tW2 = [0;0;0]; % trailer  wheel 2 vector in trailer wheel 2 COS

% transformation to robot framce (C) from wheel frames
% rotation matrices
S_W1toC = utils.CoTrafo.roty_sym(phi1); % rotation from W1 to C
S_W2toC = utils.CoTrafo.roty_sym(phi2); % rotation from W2 to C
% rotation matrices trailer
S_TtoC = utils.CoTrafo.rotz_sym(-gama); % rotation from T to C
S_tW1toT = utils.CoTrafo.roty_sym(phi1t); % rotation from tW1 to T
S_tW1toC = S_TtoC * S_tW1toT; % rotation from tW1 to C
S_tW2toT = utils.CoTrafo.roty_sym(phi2t); % rotation from tW2 to T
S_tW2toC = S_TtoC * S_tW2toT; % rotation from tW2 to C

% displacement
t_W1toC = [0;L/2;0]; % translation from W1 to C
t_W2toC = [0;-L/2; 0]; % translation from W2 to C
% trailer displacements
t_TtoG_inT = [-l_t; 0; 0];
t_GtoC_inC = [-l; 0; 0];
t_tW1toT_inT = [0; Lt/2; 0];
t_tW1toC_inT = t_tW1toT_inT + t_TtoG_inT;
t_tW2toT_inT = [0; -Lt/2; 0];
t_tW2toC_inT = t_tW2toT_inT + t_TtoG_inT;
% trailer displacements in C
t_TtoC_inC = S_TtoC*t_TtoG_inT+t_GtoC_inC;
t_tW1toC_inC = S_TtoC*t_tW1toC_inT+t_GtoC_inC;
t_tW2toC_inC = S_TtoC*t_tW2toC_inT+t_GtoC_inC;
% transformation matrices
T_W1toC = utils.CoTrafo.homog_trafo(eye(3), t_W1toC)*utils.CoTrafo.homog_trafo(S_W1toC, zeros(3,1)); % trafo from W1 to P
T_W2toC = utils.CoTrafo.homog_trafo(eye(3), t_W2toC)*utils.CoTrafo.homog_trafo(S_W2toC, zeros(3,1)); % trafo from W2 to P
%
T_TtoC = utils.CoTrafo.homog_trafo(eye(3), t_TtoC_inC)*utils.CoTrafo.homog_trafo(S_TtoC, zeros(3,1)); % trafo from T to P
T_tW1toC = utils.CoTrafo.homog_trafo(eye(3), t_tW1toC_inC)*utils.CoTrafo.homog_trafo(S_tW1toC, zeros(3,1)); % trafo from tW1 to P
T_tW2toC = utils.CoTrafo.homog_trafo(eye(3), t_tW2toC_inC)*utils.CoTrafo.homog_trafo(S_tW2toC, zeros(3,1)); % trafo from tW1 to P

% wheels depicted in robot frame (C)
p_w1_C = formula(T_W1toC*[zeros(3,1);1]); % homogeneous coordinates; formula renders symfun as vector to access entries via brackets
r_w1_C = p_w1_C(1:3); % homogeneous coordinates; 
p_w2_C = formula(T_W2toC*[zeros(3,1);1]);
r_w2_C = p_w2_C(1:3);
% trailer depicted in robot frame
p_t_C = formula(T_TtoC*[zeros(3,1);1]);
r_t_C = p_t_C(1:3);
p_tw1_C = formula(T_tW1toC*[zeros(3,1);1]);
r_tw1_C = p_tw1_C(1:3);
p_tw2_C = formula(T_tW2toC*[zeros(3,1);1]);
r_tw2_C = p_tw2_C(1:3);

% transformation to plane frame (P) from robot frame (C)
S_CtoP = utils.CoTrafo.rotz_sym(theta); % rotation from P to C
t_C2P = [xc; yc; h]; % translation from P to C
T_CtoP = utils.CoTrafo.homog_trafo(eye(3), t_C2P)*utils.CoTrafo.homog_trafo(S_CtoP, zeros(3,1)); % trafo from C to P
% T_P2C = simplify(inv(T_C2P));

% position vectors in plane frame (P)
p_c_P = formula(T_CtoP*[r_c_C;1]); % chassis
r_c_P = p_c_P(1:3);
p_w1_P = formula(T_CtoP*[r_w1_C;1]); % wheel 1 diana
r_w1_P = p_w1_P(1:3);
p_w2_P = formula(T_CtoP*[r_w2_C;1]); % wheel 2 diana
r_w2_P = p_w2_P(1:3);
% trailer
p_t_P = formula(T_CtoP*[r_t_C;1]); % chassis trailer
r_t_P = simplify(p_t_P(1:3));
p_tw1_P = formula(T_CtoP*[r_tw1_C;1]); % wheel 1 trailer
r_tw1_P = simplify(p_tw1_P(1:3));
p_tw2_P = formula(T_CtoP*[r_tw2_C;1]); % wheel 2 trailer
r_tw2_P = simplify(p_tw2_P(1:3));

% position vectors in plane frame (P)
p_c_P = formula(T_CtoP*[r_c_C;1]); % chassis
r_c_P = p_c_P(1:3);
p_w1_P = formula(T_CtoP*[r_w1_C;1]); % wheel 1 diana
r_w1_P = p_w1_P(1:3);
p_w2_P = formula(T_CtoP*[r_w2_C;1]); % wheel 2 diana
r_w2_P = p_w2_P(1:3);
% trailer
p_t_P = formula(T_CtoP*[r_t_C;1]); % chassis trailer
r_t_P = simplify(p_t_P(1:3));
p_tw1_P = formula(T_CtoP*[r_tw1_C;1]); % wheel 1 trailer
r_tw1_P = simplify(p_tw1_P(1:3));
p_tw2_P = formula(T_CtoP*[r_tw2_C;1]); % wheel 2 trailer
r_tw2_P = simplify(p_tw2_P(1:3));

%% Jacobian matrices
% translation
Jac_Transl_c = simplify(jacobian(r_c_P, y));
Jac_Transl_w1 = simplify(jacobian(r_w1_P, y));
Jac_Transl_w2 = simplify(jacobian(r_w2_P, y));
Jac_Transl_t = simplify(jacobian(r_t_P, y));
Jac_Transl_tw1 = simplify(jacobian(r_tw1_P, y));
Jac_Transl_tw2 = simplify(jacobian(r_tw2_P, y));

% rotation
T_CtoP = formula(T_CtoP);
T_W1toP = formula(T_CtoP*T_W1toC);
T_W2toP = formula(T_CtoP*T_W2toC);
T_TtoP = formula(T_CtoP*T_TtoC);
T_tW1toP = formula(T_CtoP*T_tW1toC);
T_tW2toP = formula(T_CtoP*T_tW2toC);
%
S_CtoP = T_CtoP(1:3, 1:3);
S_W1toP = T_W1toP(1:3, 1:3);
S_W2toP = T_W2toP(1:3, 1:3);
S_TtoP = simplify(T_TtoP(1:3, 1:3));
S_tW1toP = T_tW1toP(1:3, 1:3);
S_tW2toP = T_tW2toP(1:3, 1:3);
% skew-symmetric matrices to obtain Jacobi matrices of rotation
skewsym_c = simplify(diff(S_CtoP, t)*S_CtoP');
omega_c_P = utils.CoTrafo.roessel(skewsym_c);
skewsym_w1 = simplify(diff(S_W1toP, t)*S_W1toP');
omega_w1_P = utils.CoTrafo.roessel(skewsym_w1);
skewsym_w2 = simplify(diff(S_W2toP, t)*S_W2toP');
omega_w2_P = utils.CoTrafo.roessel(skewsym_w2);
skewsym_t = simplify(diff(S_TtoP, t)*S_TtoP');
omega_t_P = utils.CoTrafo.roessel(skewsym_t);
skewsym_tw1 = simplify(diff(S_tW1toP, t)*S_tW1toP');
omega_tw1_P = utils.CoTrafo.roessel(skewsym_tw1);
skewsym_tw2 = simplify(diff(S_tW2toP, t)*S_tW2toP');
omega_tw2_P = utils.CoTrafo.roessel(skewsym_tw2);
%
Jac_Rot_c = simplify(jacobian(omega_c_P, Dy));
Jac_Rot_c = subs(Jac_Rot_c, conj(y(t)), y(t));
Jac_Rot_w1 = simplify(jacobian(omega_w1_P, Dy));
Jac_Rot_w1 = simplify(subs(Jac_Rot_w1, conj(y(t)), y(t)));
Jac_Rot_w2 = simplify(jacobian(omega_w2_P, Dy));
Jac_Rot_w2 = simplify(subs(Jac_Rot_w2, conj(y(t)), y(t)));
Jac_Rot_t = simplify(jacobian(omega_t_P, Dy));
Jac_Rot_t = subs(Jac_Rot_t, conj(y(t)), y(t));
Jac_Rot_tw1 = simplify(jacobian(omega_tw1_P, Dy));
Jac_Rot_tw1 = simplify(subs(Jac_Rot_tw1, conj(y(t)), y(t)));
Jac_Rot_tw2 = simplify(jacobian(omega_tw2_P, Dy));
Jac_Rot_tw2 = simplify(subs(Jac_Rot_tw2, conj(y(t)), y(t)));

%% Nonholonomic constraint and resulting Kinematics
% transl velocities  of wheels
v_w1_NHc = formula(Jac_Transl_w1*Dy); % for testing/debugging: subs(v_w1_NHc, [dx dy dz], zeros(1,3))
v_w2_NHc = formula(Jac_Transl_w2*Dy);
v_tw1_NHc = formula(Jac_Transl_tw1*Dy);
v_tw2_NHc = formula(Jac_Transl_tw2*Dy);
% LHS of NH constraint of each wheel (rhs = 0)
NHc_w1 = simplify(v_w1_NHc(1)*sin(theta) - v_w1_NHc(2)*cos(theta));
NHc_w2 = simplify(v_w2_NHc(1)*sin(theta) - v_w2_NHc(2)*cos(theta));
NHc_tw1 = simplify(v_tw1_NHc(1)*sin(theta-gama) - v_tw1_NHc(2)*cos(theta-gama));
NHc_tw2 = simplify(v_tw2_NHc(1)*sin(theta-gama) - v_tw2_NHc(2)*cos(theta-gama));

Dy_formula = formula(Dy);
Dphi2_formula = Dy_formula(5) + L*Dy_formula(3)/R;
Dphi2t_formula = Dy_formula(6) + Lt*(Dy_formula(3)-Dy_formula(4))/R;
% "second constraint" / abrollen
NHc_w1_b = simplify(v_w1_NHc(1)*cos(theta) + v_w1_NHc(2)*sin(theta) ...
    - Dy_formula(5)*R );
NHc_w2_b = simplify(v_w2_NHc(1)*cos(theta) + v_w2_NHc(2)*sin(theta) ...
    - Dphi2_formula*R );
NHc_tw1_b = simplify(v_tw1_NHc(1)*cos(theta-gama) + v_tw1_NHc(2)*sin(theta-gama) ...
    - Dy_formula(6)*R );
NHc_tw2_b = simplify(v_tw2_NHc(1)*cos(theta-gama) + v_tw2_NHc(2)*sin(theta-gama) ...
    - Dphi2t_formula*R );
% Pfaffian constraint matrix
a1_w1 = jacobian(NHc_w1, Dy);
a1_w2 = jacobian(NHc_w2, Dy);
a1_tw1 = jacobian(NHc_tw1, Dy);
a1_tw2 = jacobian(NHc_tw2, Dy);
a2_w1 = jacobian(NHc_w1_b, Dy);
a2_w2 = jacobian(NHc_w2_b, Dy);
a2_tw1 = jacobian(NHc_tw1_b, Dy);
a2_tw2 = jacobian(NHc_tw2_b, Dy);
%
A_y = simplify([a1_w1; a2_w1; a1_w2; a2_w2; a1_tw1; a2_tw1; a1_tw2; a2_tw2]);

% explicit kinematics
% position of the middle point on the axle
r_m_P = [xc;...
    yc ;
    h];
v_m_P = simplify(diff(r_m_P, t));
s_formula = formula(s);
v_m_P_phi = simplify(subs(v_m_P, diff(theta(t),t), R*(s_formula(2) - s_formula(1))/L));
% from Anschauung
v_m_P_NH = [R*cos(theta)*(s_formula(1)+s_formula(2))/2;...
    R*sin(theta)*(s_formula(1)+s_formula(2))/2;...
    0];

% res (umstellen nach Dxc Dyc)
Res_v_m_P = simplify(v_m_P_phi - v_m_P_NH);
G_y_alt_transl = simplify(-jacobian(Res_v_m_P, s_formula));
G_y_alt_transl = formula(G_y_alt_transl);

% trafo
A_Dphi_to_v = (1/R)*[1 -0.5*L; 1 0.5*L];
%
col_Dgama_v_omega = [-sin(gama)/l_t, (l*cos(gama)+l_t)/l_t];
col_Dgama_Dphi = col_Dgama_v_omega / A_Dphi_to_v;
%
col_Dphi1t_v_omega = [1/cos(gama)-sin(gama)*tan(gama)-(Lt*sin(gama))/(2*l_t)  ...
    l*cos(gama)*tan(gama)+l*cos(gama)*Lt/(2*l_t)]/R;
col_Dphi1t_Dphi = col_Dphi1t_v_omega/A_Dphi_to_v;
col_Dphi1t_Dphi = simplify(col_Dphi1t_Dphi);

G_y = [G_y_alt_transl(1:2,:);...
    -R/L R/L;...
    col_Dgama_Dphi; ...
    [1, 0];...
    col_Dphi1t_Dphi ];
% visu: pretty(G_y_alt_transl)
% doublecheck: subs(G_y_alt, [dx dy dz], zeros(1,3))

res_check = simplify(formula(A_y)*G_y)
warning('check res check')


%% substitute parameters
robot_type = 'trailer';
run('a_config_robots');
warning('config eher als struct oder aenhliches speichern');

parameters_numeric = [M_val; L_val; I_cx_val; I_cy_val; I_cz_val;...
    Mt_val; Lt_val; I_tx_val; I_ty_val; I_tz_val;...
    m_val; R_val; I_wx_val; I_wy_val; I_wz_val;...
    l_val; l_t_val];
parameters_symbolic = [M; L; I_cx; I_cy; I_cz;...
    Mt; Lt; I_tx; I_ty; I_tz;...
    m; R; I_wx; I_wy; I_wz;...
    l; l_t];

syms xc_ yc_ theta_ theta1_ phi1_ phi1t_ real
syms Dphi1_ Dphi2_ real
y_ = [xc_ yc_ theta_ theta1_ phi1_ phi1t_]';
s_ = [Dphi1_ Dphi2_]';
x_ = [y_; s_];
xc = [yc; s];
%
x_u_ = [x_; tau_w1; tau_w2];
% x_u = [x; tau_w1; tau_w2];

% kinematics
G_y_subs = formula(subs(G_y, parameters_symbolic, parameters_numeric));
G_y_subs_no_time = subs(G_y_subs, y, y_);
% dynamics

% export ingredients of dynamics to Matlab function handles
G_y_subs_no_time = G_y_subs_no_time; % do not consider wheel angle
G_y_func = matlabFunction(G_y_subs_no_time, 'Vars', {y_(1:4)});

% trafo to v and omega
A_Dphi_to_v_val = [1/robot.R 0;0   1];
G_y_subs_no_time_v_omega = G_y_subs_no_time*A_Dphi_to_v_val;
% G_y_subs_no_time_v_omega = G_y_subs_no_time_v_omega; % do not consider wheel angle
G_y_func_v_omega = matlabFunction(G_y_subs_no_time_v_omega, 'Vars', {y_});

disp('Scucessfully computed the kinematics for articulated vehicle with inputs v and gamma_dot...');

% store the handles
save('results/EqM_articulatedVehicle_kin_vOmegaOneWheel.mat', 'G_y_func','G_y_func_v_omega',...
    'G_y_subs_no_time_v_omega', 'G_y_subs_no_time',...
    'y_','s_','x_','x_u_');
disp('Scucessfully exported the equations of motion for trailer to function handles...');

end