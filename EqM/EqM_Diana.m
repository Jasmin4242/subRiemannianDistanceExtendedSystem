% script to derive the equations of motion of diana (chassis and two
% wheels) for simulation purposes of the plant 
% mr, 28.10.2025


function EqM_Diana

% generalized coordinates
syms xc(t) yc(t) theta(t) phi1(t) phi2(t) % note that here plane=inertial frame (no tilted plane)
assume([xc(t), yc(t), theta(t), phi1(t), phi2(t)], 'real')
y = [xc; yc; theta; phi1];
assume(y(t), 'real');
Dy = diff(y,t);
% generalized velocities
syms Dphi1(t) Dphi2(t)
s = [Dphi1; Dphi2];
assume(s(t), 'real');
% geometrics
syms h L R real
% masses
syms M m real
% inertia
syms I_cx I_cy I_cz real;
syms I_wx I_wy I_wz real;
% applied forces and torques
syms g tau_w1 tau_w2 real
u = [tau_w1; tau_w2];

% 1 = left wheeel, 2 = right wheel
%% Constraint on phi2
phi2_0 = 0; % initial condition (may implement w.r.t. theta0)
phi2 = phi1 + L*theta/R + phi2_0;

%% Position vectors and transformations
% position vectors in robot (chassis frame) and wheel frame
r_c_C = [0;0;0]; % chassis vector in chassis COS
r_w1_W1 = [0;0;0]; % wheel 1 vector in Wheel 1 COS
r_w2_W2 = [0;0;0]; % wheel 2 vector in Wheel 2 COS

% transformation to robot framce (C) from wheel frames
% rotation matrices
S_W1toC = utils.CoTrafo.roty_sym(phi1); % rotation from W1 to C
S_W2toC = utils.CoTrafo.roty_sym(phi2); % rotation from W2 to C
% displacement
t_W1toC = [0;L/2;0]; % translation from W1 to C
t_W2toC = [0;-L/2; 0]; % translation from W2 to C
T_W1toC = utils.CoTrafo.homog_trafo(eye(3), t_W1toC)*utils.CoTrafo.homog_trafo(S_W1toC, zeros(3,1)); % trafo from W1 to P
T_W2toC = utils.CoTrafo.homog_trafo(eye(3), t_W2toC)*utils.CoTrafo.homog_trafo(S_W2toC, zeros(3,1)); % trafo from W2 to P
% wheels depicted in robot frame (C)
p_w1_C = formula(T_W1toC*[zeros(3,1);1]); % homogeneous coordinates; formula renders symfun as vector to access entries via brackets
r_w1_C = p_w1_C(1:3); % homogeneous coordinates; 
p_w2_C = formula(T_W2toC*[zeros(3,1);1]);
r_w2_C = p_w2_C(1:3);

% transformation to plane frame (P) from robot frame (C)
S_CtoP = utils.CoTrafo.rotz_sym(theta); % rotation from P to C
t_C2P = [xc; yc; h]; % translation from P to C
T_CtoP = utils.CoTrafo.homog_trafo(eye(3), t_C2P)*utils.CoTrafo.homog_trafo(S_CtoP, zeros(3,1)); % trafo from C to P
% T_P2C = simplify(inv(T_C2P));

% position vectors in plane frame (P)
p_c_P = formula(T_CtoP*[r_c_C;1]);
r_c_P = p_c_P(1:3);
p_w1_P = formula(T_CtoP*[r_w1_C;1]);
r_w1_P = p_w1_P(1:3);
p_w2_P = formula(T_CtoP*[r_w2_C;1]);
r_w2_P = p_w2_P(1:3);

%% Jacobian matrices
% translation
Jac_Transl_c = simplify(jacobian(r_c_P, y));
Jac_Transl_w1 = simplify(jacobian(r_w1_P, y));
Jac_Transl_w2 = simplify(jacobian(r_w2_P, y));

% rotation
T_CtoP = formula(T_CtoP);
T_W1toP = formula(T_CtoP*T_W1toC);
T_W2toP = formula(T_CtoP*T_W2toC);
S_CtoP = T_CtoP(1:3, 1:3);
S_W1toP = T_W1toP(1:3, 1:3);
S_W2toP = T_W2toP(1:3, 1:3);
% skew-symmetric matrices to obtain Jacobi matrices of rotation
skewsym_c = simplify(diff(S_CtoP, t)*S_CtoP');
omega_c_P = utils.CoTrafo.roessel(skewsym_c);
skewsym_w1 = simplify(diff(S_W1toP, t)*S_W1toP');
omega_w1_P = utils.CoTrafo.roessel(skewsym_w1);
skewsym_w2 = simplify(diff(S_W2toP, t)*S_W2toP');
omega_w2_P = utils.CoTrafo.roessel(skewsym_w2);
Jac_Rot_c = simplify(jacobian(omega_c_P, Dy));
Jac_Rot_c = subs(Jac_Rot_c, conj(y(t)), y(t));
Jac_Rot_w1 = simplify(jacobian(omega_w1_P, Dy));
Jac_Rot_w1 = simplify(subs(Jac_Rot_w1, conj(y(t)), y(t)));
Jac_Rot_w2 = simplify(jacobian(omega_w2_P, Dy));
Jac_Rot_w2 = simplify(subs(Jac_Rot_w2, conj(y(t)), y(t)));

%% Nonholonomic constraint and resulting Kinematics
% transl velocities  of wheels
v_w1_NHc = formula(Jac_Transl_w1*Dy); % for testing/debugging: subs(v_w1_NHc, [dx dy dz], zeros(1,3))
v_w2_NHc = formula(Jac_Transl_w2*Dy);
% LHS of NH constraint of each wheel (rhs = 0)
NHc_w1 = simplify(v_w1_NHc(1)*sin(theta) - v_w1_NHc(2)*cos(theta));
NHc_w2 = simplify(v_w2_NHc(1)*sin(theta) - v_w2_NHc(2)*cos(theta));

Dy_formula = formula(Dy);
Dphi2_formula = Dy_formula(4) + L*Dy_formula(3)/R;
% "second constraint" / abrollen
NHc_w1_b = simplify(v_w1_NHc(1)*cos(theta) + v_w1_NHc(2)*sin(theta) ...
    - Dy_formula(4)*R );
NHc_w2_b = simplify(v_w2_NHc(1)*cos(theta) + v_w2_NHc(2)*sin(theta) ...
    - Dphi2_formula*R );
% Pfaffian constraint matrix
a1 = jacobian(NHc_w1, Dy);
a1_w2 = jacobian(NHc_w2, Dy);
a2 = jacobian(NHc_w1_b, Dy);
a2b = jacobian(NHc_w2_b, Dy);
A_y = simplify([a1; a2; a2b]);

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
G_y = [G_y_alt_transl(1:2,:);...
    -R/L R/L;...
    [1, 0]];
% visu: pretty(G_y_alt_transl)
% doublecheck: subs(G_y_alt, [dx dy dz], zeros(1,3))

% input vector matrix
% G_y = [0.5*R*cos(theta), 0.5*R*cos(theta);...
%     0.5*R*sin(theta), 0.5*R*sin(theta);...
%     -R/L, R/L;...
%     1 0];
res_check = simplify(formula(A_y)*G_y);

%% Dynamics
% matrices L
L_Transl_c = formula(simplify(Jac_Transl_c*G_y));
L_Transl_w1 = formula(simplify(Jac_Transl_w1*G_y));
L_Transl_w2 = formula(simplify(Jac_Transl_w2*G_y));
L_Rot_c = formula(simplify(Jac_Rot_c*G_y));
L_Rot_w1 = formula(simplify(Jac_Rot_w1*G_y));
L_Rot_w1 = simplify(subs(L_Rot_w1, conj(y(t)), y(t)));
L_Rot_w2 = formula(simplify(Jac_Rot_w2*G_y));
L_Rot_w2 = simplify(subs(L_Rot_w2, conj(y(t)), y(t)));

% global mass matrix
M_c = M*eye(3);
M_w1 = m*eye(3);
M_w2 = m*eye(3);

I_c_C = diag([I_cx I_cy I_cz]);
I_w_W = diag([I_wx I_wy I_wz]);
% transformation of inertia matrix
I_c_P = S_CtoP*I_c_C*S_CtoP';
I_c_P = simplify(subs(I_c_P, conj(y(t)), y(t)));
I_w1_P = S_W1toP*I_w_W*S_W1toP';
I_w2_P = S_W2toP*I_w_W*S_W2toP';
% warning('why trafo not from wheel frame but from chassis frame sufficient')
I_w1_P = simplify(subs(I_w1_P, conj(y(t)), y(t)));
I_w2_P = simplify(subs(I_w2_P, conj(y(t)), y(t)));
%
I_w1 = I_w1_P;
I_w2 = I_w2_P;
M_bar = blkdiag(M_c, I_c_P, M_w1, I_w1, M_w2, I_w2);

% Coriolis and centrifugal
% as in holonomic systems
% a_local_c_hol = diff(Jac_Transl_c, t)*diff(y,t);
% a_local_w1_hol = diff(Jac_Transl_w1, t)*diff(y,t);
% a_local_w2_hol = diff(Jac_Transl_w2, t)*diff(y,t);
% cf. EqM nonhol
v_c = Jac_Transl_c*G_y*s;
v_w1 = Jac_Transl_w1*G_y*s;
v_w2 = Jac_Transl_w2*G_y*s;
omega_c = Jac_Rot_c*G_y*s;
omega_w1 = Jac_Rot_w1*G_y*s;
omega_w2 = Jac_Rot_w2*G_y*s;
% doublecheck
L_Transl_c_alt = jacobian( v_c, s); % aternative computation
L_Transl_w1_alt = simplify(jacobian( v_w1, s));
L_Transl_w2_alt = simplify(jacobian( v_w2, s));
% simplify(L_Transl_c - L_Transl_c_alt)
% simplify(L_Transl_w1 - L_Transl_w1_alt)
% simplify(L_Transl_w2 - L_Transl_w2_alt)

% local accelerations for nonholonomic systems
a_local_c = simplify(jacobian(v_c, y)*Dy);
a_local_w1 = jacobian(v_w1, y)*Dy;
a_local_w2 = jacobian(v_w2, y)*Dy;
alpha_local_c = simplify(jacobian(omega_c, y)*Dy);
alpha_local_w1 = jacobian(omega_w1, y)*Dy;
alpha_local_w2 = jacobian(omega_w2, y)*Dy;
% doublechek
a_local_c_alt = simplify(diff(L_Transl_c, t)*s);
a_local_w1_alt = simplify(diff(L_Transl_w1, t)*s);
a_local_w2_alt = simplify(diff(L_Transl_w2, t)*s);
alpha_local_c_alt = simplify(diff(L_Rot_c, t)*s);
alpha_local_w1_alt = simplify(diff(L_Rot_w1, t)*s);
alpha_local_w2_alt = simplify(diff(L_Rot_w2, t)*s);
% a_local_c - a_local_c_alt
% a_local_w1 - a_local_w1_alt
% a_local_w2 - a_local_w2_alt
% assume(theta(t), 'real')
res_alpha_c = simplify(alpha_local_c - alpha_local_c_alt);
res_alpha_c = subs(res_alpha_c, conj(theta(t)), theta(t));  % manually assume real
res_alpha_w1 =  simplify(alpha_local_w1 - alpha_local_w1_alt);
res_alpha_w1 = subs(res_alpha_w1, conj(y(t)), y(t));
res_alpha_w1 = simplify(subs(res_alpha_w1, conj(diff(y(t), t)), diff(y(t), t)));
res_alpha_w2 = simplify(alpha_local_w2 - alpha_local_w2_alt);
res_alpha_w2 = subs(res_alpha_w2, conj(y(t)), y(t));
res_alpha_w2 = simplify(subs(res_alpha_w2, conj(diff(y(t), t)), diff(y(t), t)));

% vector of Coriolis and centrifugal
omega_local_c = zeros(3,1);
omega_local_w1 = zeros(3,1);
omega_local_w2 = zeros(3,1);
k_c = [M*a_local_c; I_c_P*alpha_local_c + skewsym_c*I_c_P*omega_local_c];
k_w1 = [m*a_local_w1; I_w1*alpha_local_w1 + skewsym_w1*I_w1*omega_local_w1];
k_w2 = [m*a_local_w2; I_w2*alpha_local_w2 + skewsym_w2*I_w2*omega_local_w2];
k_bar = [k_c; k_w1; k_w2];
k_bar = subs(k_bar, conj(y(t)), y(t));
k_bar = simplify(subs(k_bar, conj(diff(y(t), t)), diff(y(t), t)));

% applied forces and torques
f_appl_c = [0; 0; -M*g];
f_appl_w1 = [0; 0; -m*g];
f_appl_w2 = [0; 0; -m*g];
ell_appl_c = zeros(3,1);
ell_appl_w1 = S_W1toP*[0; tau_w1; 0];
ell_appl_w2 = S_W2toP*[0; tau_w2; 0];
% resulting vector
qbar_a = [f_appl_c; ell_appl_c; f_appl_w1; ell_appl_w1; f_appl_w2; ell_appl_w2];
qbar_a = subs(qbar_a, conj(y(t)), y(t));
qbar_a = simplify(subs(qbar_a, conj(diff(y(t), t)), diff(y(t), t)));

% (Jacobian) velocity matrix
L_bar = [L_Transl_c; L_Rot_c; L_Transl_w1; L_Rot_w1; L_Transl_w2; L_Rot_w2];
L_bar = subs(L_bar, conj(y(t)), y(t));
L_bar = simplify(subs(L_bar, conj(diff(y(t), t)), diff(y(t), t)));

%% Equations of Motion
M_eqM = simplify(L_bar'*M_bar*L_bar);
M_eqM = subs(M_eqM, conj(y(t)), y(t));
M_eqM = simplify(subs(M_eqM, conj(diff(y(t), t)), diff(y(t), t)));
M_eqM = simplify(subs(M_eqM, I_wx, I_wz));
%
k_eqM = simplify(L_bar'*k_bar);
k_eqM = subs(k_eqM, conj(y(t)), y(t));
k_eqM = simplify(subs(k_eqM, conj(diff(y(t), t)), diff(y(t), t)));
%
q_eqM = simplify(L_bar'*qbar_a);
q_eqM = subs(q_eqM, conj(y(t)), y(t));
q_eqM = simplify(subs(q_eqM, conj(diff(y(t), t)), diff(y(t), t)));


%% substitute parameters
robot_type = 'diana';
run('a_config_robots');
warning('config eher als struct oder aenhliches speichern');

parameters_numeric = [M_val; L_val; I_cx_val; I_cy_val; I_cz_val;...
    m_val; R_val; I_wx_val; I_wy_val; I_wz_val];
parameters_symbolic = [M; L; I_cx; I_cy; I_cz;...
    m; R; I_wx; I_wy; I_wz];
syms xc_ yc_ theta_ phi1_ real
syms Dphi1_ Dphi2_ real
y_ = [xc_ yc_ theta_ phi1_]';
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
G_y_func = matlabFunction(G_y_subs_no_time, 'Vars', {y_});
M_eqM_func = matlabFunction(M_eqM_subs_no_time, 'Vars', {y_});
Minv_eqM_func = matlabFunction(Minv_eqM_subs_no_time, 'Vars', {y_});
k_eqM_func = matlabFunction(k_eqM_subs_no_time, 'Vars', {x_});
q_eqM_func = matlabFunction(q_eqM_subs_no_time, 'Vars', {x_u_});

% trafo to v and omega
A_Dphi_to_v = (1/robot.R)*[1 -0.5*robot.L; 1 0.5*robot.L];
G_y_subs_no_time_v_omega = G_y_subs_no_time*A_Dphi_to_v;
G_y_subs_no_time_v_omega = G_y_subs_no_time_v_omega; % do not consider wheel angle
G_y_func_v_omega = matlabFunction(G_y_subs_no_time_v_omega, 'Vars', {y_});
M_eqM_func_vw = matlabFunction(M_eqM_subs_no_time*A_Dphi_to_v, 'Vars', {y_});
Minv_eqM_func_vw = matlabFunction(inv(A_Dphi_to_v)*Minv_eqM_subs_no_time, 'Vars', {y_});


disp('Scucessfully computed the equations of motion for Diana...');

% store the handles
save('results/EqM_Diana.mat', 'M_eqM_func','M_eqM_func_vw','Minv_eqM_func','Minv_eqM_func_vw','k_eqM_func', 'q_eqM_func','G_y_func','G_y_func_v_omega',...
    'y_','s_','x_','x_u_');
disp('Scucessfully exported the equations of motion for Diana to function handles...');

end