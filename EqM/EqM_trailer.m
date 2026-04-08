% script to derive the equations of motion of robot-trailer system 
% for simulation purposes of the plant 
% mr, 3.11.2025


function EqM_trailer

% generalized coordinates
syms xc(t) yc(t) theta(t) phi1(t) phi2(t) xt(t) yt(t) theta1(t) phi1t(t) phi2t(t) % note that here plane=inertial frame (no tilted plane)
assume([xc(t), yc(t), theta(t), phi1(t), phi2(t), ...
    xt(t), yt(t), theta1(t), phi1t(t), phi2t(t)], 'real')
y = [xc; yc; theta; theta1; phi1; phi1t];
assume(y(t), 'real');
Dy = diff(y,t);
% generalized velocities
syms Dphi1(t) Dphi2(t)
s = [Dphi1; Dphi2];
assume(s(t), 'real');
% geometrics
syms h L Lt R b1x real % LT real
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

% 1 = left wheeel, 2 = right wheel
%% Constraint on phi2 and phi2t
phi2_0 = 0; % initial condition (may implement w.r.t. theta0)
phi2 = phi1 + L*theta/R + phi2_0;
phi2t_0 = 0; % initial condition (may implement w.r.t. theta0)
phi2t = phi1t + Lt*theta1/R + phi2t_0;

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
S_TtoC = utils.CoTrafo.rotz_sym(theta1 - theta); % rotation from T to C
S_tW1toT = utils.CoTrafo.roty_sym(phi1t); % rotation from tW1 to T
S_tW1toC = S_TtoC * S_tW1toT; % rotation from tW1 to C
S_tW2toT = utils.CoTrafo.roty_sym(phi2t); % rotation from tW2 to T
S_tW2toC = S_TtoC * S_tW2toT; % rotation from tW2 to C

% displacement
t_W1toC = [0;L/2;0]; % translation from W1 to C
t_W2toC = [0;-L/2; 0]; % translation from W2 to C
% trailer displacements
t_TtoC_inT = [-b1x; 0; 0];
t_tW1toT_inT = [0; Lt/2; 0];
t_tW1toC_inT = t_tW1toT_inT + t_TtoC_inT;
t_tW2toT_inT = [0; -Lt/2; 0];
t_tW2toC_inT = t_tW2toT_inT + t_TtoC_inT;
% trailer displacements in C
t_TtoC_inC = S_TtoC*t_TtoC_inT;
t_tW1toC_inC = S_TtoC*t_tW1toC_inT;
t_tW2toC_inC = S_TtoC*t_tW2toC_inT;
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
NHc_tw1 = simplify(v_tw1_NHc(1)*sin(theta1) - v_tw1_NHc(2)*cos(theta1));
NHc_tw2 = simplify(v_tw2_NHc(1)*sin(theta1) - v_tw2_NHc(2)*cos(theta1));

Dy_formula = formula(Dy);
Dphi2_formula = Dy_formula(5) + L*Dy_formula(3)/R;
Dphi2t_formula = Dy_formula(6) + Lt*Dy_formula(4)/R;
% "second constraint" / abrollen
NHc_w1_b = simplify(v_w1_NHc(1)*cos(theta) + v_w1_NHc(2)*sin(theta) ...
    - Dy_formula(5)*R );
NHc_w2_b = simplify(v_w2_NHc(1)*cos(theta) + v_w2_NHc(2)*sin(theta) ...
    - Dphi2_formula*R );
NHc_tw1_b = simplify(v_tw1_NHc(1)*cos(theta1) + v_tw1_NHc(2)*sin(theta1) ...
    - Dy_formula(6)*R );
NHc_tw2_b = simplify(v_tw2_NHc(1)*cos(theta1) + v_tw2_NHc(2)*sin(theta1) ...
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
A_y = simplify([a1_w1; a2_w1; a2_w2]);
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
col_Dtheta1_v_omega = [sin(theta-theta1)/b1x 0];
col_Dtheta1_Dphi = col_Dtheta1_v_omega / (A_Dphi_to_v);
%
col_Dphi1t_v_omega = [cos(theta-theta1)/R - Lt*sin(theta-theta1)/(2*b1x*R), 0];
col_Dphi1t_Dphi = col_Dphi1t_v_omega / (A_Dphi_to_v);

G_y = [G_y_alt_transl(1:2,:);...
    -R/L R/L;...
    col_Dtheta1_Dphi; ...
    [1, 0];...
    col_Dphi1t_Dphi ];
% visu: pretty(G_y_alt_transl)
% doublecheck: subs(G_y_alt, [dx dy dz], zeros(1,3))

res_check = simplify(formula(A_y)*G_y)
warning('check res check')

%% Dynamics
% matrices L
L_Transl_c = formula(simplify(Jac_Transl_c*G_y));
L_Transl_t = formula(simplify(Jac_Transl_t*G_y));
L_Transl_w1 = formula(simplify(Jac_Transl_w1*G_y));
L_Transl_w2 = formula(simplify(Jac_Transl_w2*G_y));
L_Transl_tw1 = formula(simplify(Jac_Transl_tw1*G_y));
L_Transl_tw2 = formula(simplify(Jac_Transl_tw2*G_y));
L_Rot_c = formula(simplify(Jac_Rot_c*G_y));
L_Rot_t = formula(simplify(Jac_Rot_t*G_y));
L_Rot_w1 = formula(simplify(Jac_Rot_w1*G_y));
L_Rot_w1 = simplify(subs(L_Rot_w1, conj(y(t)), y(t)));
L_Rot_w2 = formula(simplify(Jac_Rot_w2*G_y));
L_Rot_w2 = simplify(subs(L_Rot_w2, conj(y(t)), y(t)));
L_Rot_tw1 = formula(simplify(Jac_Rot_tw1*G_y));
L_Rot_tw1 = simplify(subs(L_Rot_tw1, conj(y(t)), y(t)));
L_Rot_tw2 = formula(simplify(Jac_Rot_tw2*G_y));
L_Rot_tw2 = simplify(subs(L_Rot_tw2, conj(y(t)), y(t)));

% global mass matrix
M_c = M*eye(3);
M_t = Mt*eye(3);
M_w1 = m*eye(3);
M_w2 = m*eye(3);
M_tw1 = m*eye(3);
M_tw2 = m*eye(3);

I_c_C = diag([I_cx I_cy I_cz]);
I_t_T = diag([I_tx I_ty I_tz]);
I_w_W = diag([I_wx I_wy I_wz]);
% transformation of inertia matrix
I_c_P = S_CtoP*I_c_C*S_CtoP';
I_c_P = simplify(subs(I_c_P, conj(y(t)), y(t)));
%
I_t_P = S_TtoP*I_t_T*S_TtoP';
I_t_P = simplify(subs(I_t_P, conj(y(t)), y(t)));
%
I_w1_P = S_W1toP*I_w_W*S_W1toP';
I_w2_P = S_W2toP*I_w_W*S_W2toP';
% warning('why trafo not from wheel frame but from chassis frame sufficient')
I_w1_P = simplify(subs(I_w1_P, conj(y(t)), y(t)));
I_w2_P = simplify(subs(I_w2_P, conj(y(t)), y(t)));
%
I_tw1_P = S_tW1toP*I_w_W*S_tW1toP';
I_tw2_P = S_tW2toP*I_w_W*S_tW2toP';
% warning('why trafo not from wheel frame but from chassis frame sufficient')
I_tw1_P = simplify(subs(I_tw1_P, conj(y(t)), y(t)));
I_tw2_P = simplify(subs(I_tw2_P, conj(y(t)), y(t)));
%
I_w1 = I_w1_P;
I_w2 = I_w2_P;
I_tw1 = I_tw1_P;
I_tw2 = I_tw2_P;
M_bar = blkdiag(M_c, I_c_P, M_t, I_t_P,...
    M_w1, I_w1, M_w2, I_w2, M_tw1, I_tw1, M_tw2, I_tw2);

% Coriolis and centrifugal
% as in holonomic systems
% a_local_c_hol = diff(Jac_Transl_c, t)*diff(y,t);
% a_local_w1_hol = diff(Jac_Transl_w1, t)*diff(y,t);
% a_local_w2_hol = diff(Jac_Transl_w2, t)*diff(y,t);
% cf. EqM nonhol
v_c = Jac_Transl_c*G_y*s;
v_t = Jac_Transl_t*G_y*s;
v_w1 = Jac_Transl_w1*G_y*s;
v_w2 = Jac_Transl_w2*G_y*s;
v_tw1 = Jac_Transl_tw1*G_y*s;
v_tw2 = Jac_Transl_tw2*G_y*s;
omega_c = Jac_Rot_c*G_y*s;
omega_t = Jac_Rot_t*G_y*s;
omega_w1 = Jac_Rot_w1*G_y*s;
omega_w2 = Jac_Rot_w2*G_y*s;
omega_tw1 = Jac_Rot_tw1*G_y*s;
omega_tw2 = Jac_Rot_tw2*G_y*s;
% doublecheck
L_Transl_c_alt = jacobian( v_c, s); % aternative computation
L_Transl_t_alt = jacobian( v_t, s); % aternative computation
L_Transl_w1_alt = simplify(jacobian( v_w1, s));
L_Transl_w2_alt = simplify(jacobian( v_w2, s));
L_Transl_tw1_alt = simplify(jacobian( v_tw1, s));
L_Transl_tw2_alt = simplify(jacobian( v_tw2, s));
% simplify(L_Transl_c - L_Transl_c_alt)
% simplify(L_Transl_t - L_Transl_t_alt)
% simplify(L_Transl_w1 - L_Transl_w1_alt)
% simplify(L_Transl_w2 - L_Transl_w2_alt)
% simplify(L_Transl_tw1 - L_Transl_tw1_alt)
% simplify(L_Transl_tw2 - L_Transl_tw2_alt)

% local accelerations for nonholonomic systems
a_local_c = simplify(jacobian(v_c, y)*Dy);
a_local_t = simplify(jacobian(v_t, y)*Dy);
a_local_w1 = jacobian(v_w1, y)*Dy;
a_local_w2 = jacobian(v_w2, y)*Dy;
a_local_tw1 = jacobian(v_tw1, y)*Dy;
a_local_tw2 = jacobian(v_tw2, y)*Dy;
alpha_local_c = simplify(jacobian(omega_c, y)*Dy);
alpha_local_t = simplify(jacobian(omega_t, y)*Dy);
alpha_local_w1 = jacobian(omega_w1, y)*Dy;
alpha_local_w2 = jacobian(omega_w2, y)*Dy;
alpha_local_tw1 = jacobian(omega_tw1, y)*Dy;
alpha_local_tw2 = jacobian(omega_tw2, y)*Dy;
% doublechek
a_local_c_alt = simplify(diff(L_Transl_c, t)*s);
a_local_t_alt = simplify(diff(L_Transl_t, t)*s);
a_local_w1_alt = simplify(diff(L_Transl_w1, t)*s);
a_local_w2_alt = simplify(diff(L_Transl_w2, t)*s);
a_local_tw1_alt = simplify(diff(L_Transl_tw1, t)*s);
a_local_tw2_alt = simplify(diff(L_Transl_tw2, t)*s);
alpha_local_c_alt = simplify(diff(L_Rot_c, t)*s);
alpha_local_t_alt = simplify(diff(L_Rot_t, t)*s);
alpha_local_w1_alt = simplify(diff(L_Rot_w1, t)*s);
alpha_local_w2_alt = simplify(diff(L_Rot_w2, t)*s);
alpha_local_tw1_alt = simplify(diff(L_Rot_tw1, t)*s);
alpha_local_tw2_alt = simplify(diff(L_Rot_tw2, t)*s);
% a_local_c - a_local_c_alt
% a_local_t - a_local_t_alt
% a_local_w1 - a_local_w1_alt
% a_local_w2 - a_local_w2_alt
% simplify(a_local_tw1 - a_local_tw1_alt)
% simplify(a_local_tw2 - a_local_tw2_alt)

% assume(theta(t), 'real')
res_alpha_c = simplify(alpha_local_c - alpha_local_c_alt);
res_alpha_c = subs(res_alpha_c, conj(theta(t)), theta(t));  % manually assume real
res_alpha_t = simplify(alpha_local_t - alpha_local_t_alt);
res_alpha_t = subs(res_alpha_t, conj(theta(t)), theta(t));
%
res_alpha_w1 =  simplify(alpha_local_w1 - alpha_local_w1_alt);
res_alpha_w1 = subs(res_alpha_w1, conj(y(t)), y(t));
res_alpha_w1 = simplify(subs(res_alpha_w1, conj(diff(y(t), t)), diff(y(t), t)));
res_alpha_w2 = simplify(alpha_local_w2 - alpha_local_w2_alt);
res_alpha_w2 = subs(res_alpha_w2, conj(y(t)), y(t));
res_alpha_w2 = simplify(subs(res_alpha_w2, conj(diff(y(t), t)), diff(y(t), t)));
%
res_alpha_tw1 =  simplify(alpha_local_tw1 - alpha_local_tw1_alt);
res_alpha_tw1 = subs(res_alpha_tw1, conj(y(t)), y(t));
res_alpha_tw1 = simplify(subs(res_alpha_tw1, conj(diff(y(t), t)), diff(y(t), t)));
res_alpha_tw2 = simplify(alpha_local_tw2 - alpha_local_tw2_alt);
res_alpha_tw2 = subs(res_alpha_tw2, conj(y(t)), y(t));
res_alpha_tw2 = simplify(subs(res_alpha_tw2, conj(diff(y(t), t)), diff(y(t), t)));

% vector of Coriolis and centrifugal
omega_local_c = zeros(3,1);
omega_local_t = zeros(3,1);
omega_local_w1 = zeros(3,1);
omega_local_w2 = zeros(3,1);
omega_local_tw1 = zeros(3,1);
omega_local_tw2 = zeros(3,1);
k_c = [M*a_local_c; I_c_P*alpha_local_c + skewsym_c*I_c_P*omega_local_c];
k_t = [Mt*a_local_t; I_t_P*alpha_local_t + skewsym_t*I_t_P*omega_local_t];
k_w1 = [m*a_local_w1; I_w1*alpha_local_w1 + skewsym_w1*I_w1*omega_local_w1];
k_w2 = [m*a_local_w2; I_w2*alpha_local_w2 + skewsym_w2*I_w2*omega_local_w2];
k_tw1 = [m*a_local_tw1; I_tw1*alpha_local_tw1 + skewsym_tw1*I_tw1*omega_local_tw1];
k_tw2 = [m*a_local_tw2; I_tw2*alpha_local_tw2 + skewsym_tw2*I_tw2*omega_local_tw2];
k_bar = [k_c; k_t; k_w1; k_w2;  k_tw1; k_tw2];
k_bar = subs(k_bar, conj(y(t)), y(t));
k_bar = simplify(subs(k_bar, conj(diff(y(t), t)), diff(y(t), t)));

% applied forces and torques
f_appl_c = [0; 0; -M*g];
f_appl_t = [0; 0; -Mt*g];
f_appl_w1 = [0; 0; -m*g];
f_appl_w2 = [0; 0; -m*g];
f_appl_tw1 = [0; 0; -m*g];
f_appl_tw2 = [0; 0; -m*g];
ell_appl_c = zeros(3,1);
ell_appl_t = zeros(3,1);
ell_appl_w1 = S_CtoP*S_W1toC*[0; tau_w1; 0];
ell_appl_w2 = S_CtoP*S_W2toC*[0; tau_w2; 0];
ell_appl_tw1 = zeros(3,1);
ell_appl_tw2 = zeros(3,1);
% resulting vector
qbar_a = [f_appl_c; ell_appl_c; f_appl_t; ell_appl_t;...
    f_appl_w1; ell_appl_w1; f_appl_w2; ell_appl_w2;...
    f_appl_tw1; ell_appl_tw1; f_appl_tw2; ell_appl_tw2];
qbar_a = subs(qbar_a, conj(y(t)), y(t));
qbar_a = simplify(subs(qbar_a, conj(diff(y(t), t)), diff(y(t), t)));

% (Jacobian) velocity matrix
L_bar = [L_Transl_c; L_Rot_c; L_Transl_t; L_Rot_t;...
    L_Transl_w1; L_Rot_w1; L_Transl_w2; L_Rot_w2;...
    L_Transl_tw1; L_Rot_tw1; L_Transl_tw2; L_Rot_tw2];
L_bar = subs(L_bar, conj(y(t)), y(t));
L_bar = simplify(subs(L_bar, conj(diff(y(t), t)), diff(y(t), t)));

%% Equations of Motion
M_eqM = simplify(L_bar'*M_bar*L_bar);
M_eqM = subs(M_eqM, conj(y(t)), y(t));
M_eqM = simplify(subs(M_eqM, conj(diff(y(t), t)), diff(y(t), t)));
%
k_eqM = simplify(L_bar'*k_bar);
k_eqM = subs(k_eqM, conj(y(t)), y(t));
k_eqM = simplify(subs(k_eqM, conj(diff(y(t), t)), diff(y(t), t)));
%
q_eqM = simplify(L_bar'*qbar_a);
q_eqM = subs(q_eqM, conj(y(t)), y(t));
q_eqM = simplify(subs(q_eqM, conj(diff(y(t), t)), diff(y(t), t)));


%% substitute parameters
robot_type = 'trailer';
run('a_config_robots');
warning('config eher als struct oder aenhliches speichern');

parameters_numeric = [M_val; L_val; I_cx_val; I_cy_val; I_cz_val;...
    Mt_val; Lt_val; I_tx_val; I_ty_val; I_tz_val;...
    m_val; R_val; I_wx_val; I_wy_val; I_wz_val;
    b1x_val];
parameters_symbolic = [M; L; I_cx; I_cy; I_cz;...
    Mt; Lt; I_tx; I_ty; I_tz;...
    m; R; I_wx; I_wy; I_wz;...
    b1x];

syms xc_ yc_ theta_ theta1_ phi1_ phi1t_ real
syms Dphi1_ Dphi2_ real
y_ = [xc_ yc_ theta_ theta1_ phi1_ phi1t_]';
s_ = [Dphi1_ Dphi2_]';
x_ = [y_; s_];
x = [y; s];
%
x_u_ = [x_; tau_w1; tau_w2];
% x_u = [x; tau_w1; tau_w2];

% kinematics
G_y_subs = formula(subs(G_y, parameters_symbolic, parameters_numeric));
G_y_subs_no_time = subs(G_y_subs, y, y_);
% dynamics
M_eqM_subs = formula(subs(M_eqM, parameters_symbolic, parameters_numeric));
M_eqM_subs_no_time = subs(M_eqM_subs, y, y_);
Minv_eqM_subs_no_time = inv(M_eqM_subs_no_time);
k_eqM_subs = formula(subs(k_eqM, parameters_symbolic, parameters_numeric));
k_eqM_subs_no_time = subs(k_eqM_subs, x, x_);
q_eqM_subs = formula(subs(q_eqM, parameters_symbolic, parameters_numeric));
q_eqM_subs_no_time = subs(q_eqM_subs, x, x_);

% export ingredients of dynamics to Matlab function handles
G_y_subs_no_time = G_y_subs_no_time; % do not consider wheel angle
G_y_func = matlabFunction(G_y_subs_no_time, 'Vars', {y_(1:4)});
M_eqM_func = matlabFunction(M_eqM_subs_no_time, 'Vars', {y_});
Minv_eqM_func = matlabFunction(Minv_eqM_subs_no_time, 'Vars', {y_});
k_eqM_func = matlabFunction(k_eqM_subs_no_time, 'Vars', {x_});
q_eqM_func = matlabFunction(q_eqM_subs_no_time, 'Vars', {x_u_});

% trafo to v and omega
A_Dphi_to_v_val = (1/robot.R)*[1 -0.5*robot.L; 1 0.5*robot.L];
G_y_subs_no_time_v_omega = G_y_subs_no_time*A_Dphi_to_v_val;
% G_y_subs_no_time_v_omega = G_y_subs_no_time_v_omega; % do not consider wheel angle
G_y_func_v_omega = matlabFunction(G_y_subs_no_time_v_omega, 'Vars', {y_});


disp('Scucessfully computed the equations of motion for trailer system...');

% store the handles
save('results/EqM_trailer.mat', 'M_eqM_func','Minv_eqM_func','k_eqM_func', 'q_eqM_func','G_y_func','G_y_func_v_omega',...
    'G_y_subs_no_time_v_omega', 'G_y_subs_no_time', 'M_eqM_subs_no_time', 'k_eqM_subs_no_time', 'q_eqM_subs_no_time',...
    'y_','s_','x_','x_u_');
disp('Scucessfully exported the equations of motion for trailer to function handles...');

end