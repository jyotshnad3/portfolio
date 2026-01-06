% -------------------------------------------------------------------------
% Model Reference Adaptive Control (MRAC) vs Fixed MRC
% Relative Degree n* = 2 Approach with Normalization
% -------------------------------------------------------------------------

clear; clc; close all;

% 1. Simulation & Vehicle Parameters
t_change = 8.0; % Time at which cornering stiffness drops by 50% [s]

m  = 1500;    % Mass [kg]
Iz = 2500;    % Yaw Moment of Inertia [kg m^2]
Ca_f = 55000; % Nominal Front Cornering Stiffness [N/rad]
Ca_r = 60000; % Nominal Rear Cornering Stiffness [N/rad]
a  = 1.2;     % CG to Front Axle [m]
b  = 1.4;     % CG to Rear Axle [m]
u  = 20;      % Longitudinal Velocity [m/s]

% Pack vehicle parameters for simulation
vp.m = m; vp.Iz = Iz; vp.Ca_f = Ca_f; vp.Ca_r = Ca_r;
vp.a = a; vp.b = b; vp.u = u; vp.t_change = t_change;

% 2. Nominal Plant Model Setup (For Initialization)
% States: x = [y; y_dot; psi; r]
% Input: delta_f
% Output: y

% A matrix (Nominal)
a22 = -(Ca_f + Ca_r) / (m * u);
a23 = (Ca_f + Ca_r) / m;
a24 = (b * Ca_r - a * Ca_f) / (m * u);
a42 = (b * Ca_r - a * Ca_f) / (Iz * u);
a43 = (a * Ca_f - b * Ca_r) / Iz;
a44 = -(a^2 * Ca_f + b^2 * Ca_r) / (Iz * u);

b2 = Ca_f / m;
b4 = a * Ca_f / Iz;

A_p = [0,  1,   0,   0;
       0, a22, a23, a24;
       0,  0,   0,   1;
       0, a42, a43, a44];

B_p = [0; b2; 0; b4];
C_p = [1, 0, 0, 0];
D_p = 0;

n = 4; % Plant Order

% Transfer Function
[num_p, den_p] = ss2tf(A_p, B_p, C_p, D_p);
num_p(abs(num_p) < 1e-10) = 0;
den_p(abs(den_p) < 1e-10) = 0;

first_nz_idx = find(abs(num_p) > 1e-10, 1);
k_p = num_p(first_nz_idx);
Z_p = num_p(first_nz_idx:end) / k_p;
P_p = den_p;

% 3. Reference Model Setup
% Wm(s) = 1 / (s + 3)^2
Z_m = 1; 
P_m = [1, 6, 9];
k_m = P_m(3);
p0 = P_m(2)/2; % Pole for n*=2 design (Wm = M(s)/(s+p0) -> M(s) must be SPR)

% 4. MRC Initialization (Solving for Theta*)
% Filter Lambda(s) = (s+10)^3 = s^3 + 30s^2 + 300s + 1000
Lambda = poly([-3, -3, -3]); 


% Diophantine Equation Solution
c_0_star = k_m / k_p;

RHS_poly = conv(conv(Z_p, Lambda), P_m) - conv(Lambda, P_p);
RHS_vec = [zeros(1, 8 - length(RHS_poly)), RHS_poly]'; 

M = zeros(8, 7);
term_t1_1 = - [P_p, 0, 0]; 
term_t1_2 = - [0, P_p, 0];
term_t1_3 = - [0, 0, P_p];
M(:, 1) = [zeros(8-length(term_t1_1),1); term_t1_1'];
M(:, 2) = [zeros(8-length(term_t1_2),1); term_t1_2'];
M(:, 3) = [zeros(8-length(term_t1_3),1); term_t1_3'];

KZp = k_p * Z_p;
term_t2_1 = - conv(KZp, [1, 0, 0]); 
term_t2_2 = - conv(KZp, [1, 0]);    
term_t2_3 = - KZp;                  
M(:, 4) = [zeros(8-length(term_t2_1),1); term_t2_1'];
M(:, 5) = [zeros(8-length(term_t2_2),1); term_t2_2'];
M(:, 6) = [zeros(8-length(term_t2_3),1); term_t2_3'];

term_t3 = - conv(KZp, Lambda);
M(:, 7) = [zeros(8-length(term_t3),1); term_t3'];

mrc_params = pinv(M) * RHS_vec;

theta_1_star = mrc_params(1:3);
theta_2_star = mrc_params(4:6);
theta_3_star = mrc_params(7);

% Construct Initial Parameter Vector Theta_0
% Order in code: [theta_1; theta_2; theta_3; c_0]
theta_init = [theta_1_star; theta_2_star; theta_3_star; c_0_star];

fprintf('MRC Parameters (Used for Initialization):\n');
disp(theta_init);

% 5. Simulation Setup
% Filter Matrices (F, g) for alpha(s)/Lambda(s)
l2 = Lambda(2); l1 = Lambda(3); l0 = Lambda(4);
F = [-l2, -l1, -l0;
      1,   0,   0;
      0,   1,   0];
g = [1; 0; 0];

% Reference Model State Space
[A_m, B_m, C_m, D_m] = tf2ss(k_m*Z_m, P_m);

% Simulation Time
T_sim = 15;
t_span = [0 T_sim];

% Initial Conditions (Common for both)
X0 = zeros(28, 1);
X0(21:28) = theta_init; % Initialize parameters

% Options
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% --- SIMULATION 1: MRAC (Adapting) ---
% Gamma_mrac = 200 * eye(8); 
Gamma_mrac = 20 * eye(8); 
[T_mrac, X_mrac] = ode45(@(t, x) sys_dynamics(t, x, vp, F, g, A_m, B_m, C_m, D_m, ...
    Gamma_mrac, p0, sign(c_0_star)), t_span, X0, options);

% --- SIMULATION 2: Fixed MRC (Non-Adapting) ---
% We simply set Gamma to zero. This keeps theta constant at theta_init.
Gamma_mrc = zeros(8);
[T_mrc, X_mrc] = ode45(@(t, x) sys_dynamics(t, x, vp, F, g, A_m, B_m, C_m, D_m, ...
    Gamma_mrc, p0, sign(c_0_star)), t_span, X0, options);

% 6. Plotting & Analysis
% Helper function to extract signals
[y_p_mrac, u_mrac, r_mrac, y_m_mrac] = process_results(T_mrac, X_mrac, C_m, D_m, Gamma_mrac, c_0_star);
[y_p_mrc, u_mrc, r_mrc, y_m_mrc]   = process_results(T_mrc, X_mrc, C_m, D_m, Gamma_mrc, c_0_star);
figure('Color','w', 'Position', [100 100 1000 800]);
subplot(3,1,1);
plot(T_mrac, r_mrac, 'g:', 'LineWidth', 1.5); hold on;
plot(T_mrac, y_m_mrac, 'k--', 'LineWidth', 2);
plot(T_mrc, y_p_mrc, 'b-.', 'LineWidth', 1.5);
plot(T_mrac, y_p_mrac, 'r-', 'LineWidth', 2);
xline(t_change, 'k-', 'Alpha Change');
grid on;
legend('Reference r(t)', 'Ref Model y_m', 'Fixed MRC y_p', 'Adaptive MRAC y_p', 'Location', 'Best');
ylabel('Lateral Deviation [m]');
title('Tracking Performance: MRAC vs Fixed MRC');

subplot(3,1,2);
plot(T_mrc, rad2deg(u_mrc), 'b-.', 'LineWidth', 1.5); hold on;
plot(T_mrac, rad2deg(u_mrac), 'r-', 'LineWidth', 1.5);
xline(t_change, 'k-');
grid on;
legend('Fixed MRC', 'Adaptive MRAC');
ylabel('Steering [deg]');
title('Control Input (No Saturation)');

subplot(3,1,3);
% Plotting only MRAC parameters (MRC are constant)
theta_hist = X_mrac(:, 21:28);
hold on
for i = 1:8
    plot(T_mrac,theta_hist(:,i))
end
xline(t_change, 'k-');
grid on;
ylabel('Parameter Values');
xlabel('Time [s]');
title('MRAC Parameter Adaptation');
% legend('\theta_1', '', '', '\theta_2', '', '', '\theta_3', 'c_0');
legend
hold off


%% Auxiliary Functions

function [y_p, u_control, r_trace, y_m] = process_results(T, X, C_m, D_m, Gamma, c0_star)
    y_p = X(:, 1);
    y_m = zeros(length(T), 1);
    u_control = zeros(length(T), 1);
    r_trace = zeros(length(T), 1);
    
    for i = 1:length(T)
        t_curr = T(i);
        r_val = reference_signal(t_curr);
        r_trace(i) = r_val;
        
        % Ref Model Output
        x_m_curr = X(i, 11:12)';
        y_m(i) = C_m * x_m_curr + D_m * r_val;
        
        % Reconstruct Control
        omega_1 = X(i, 5:7)';
        omega_2 = X(i, 8:10)';
        phi = X(i, 13:20)';
        theta = X(i, 21:28)';
        
        omega = [omega_1; omega_2; y_p(i); r_val];
        e1 = y_p(i) - y_m(i);
        norm_factor = 1 + phi' * phi;
        
        correction_term = (e1 * sign(c0_star) * (phi' * Gamma * phi)) / norm_factor;
        u_val = theta' * omega - correction_term;
        
        % No Saturation here!
        u_control(i) = u_val;
    end
end

function dX = sys_dynamics(t, X, vp, F, g, A_m, B_m, C_m, D_m, ...
                            Gamma, p0, sgn_c0)
    
    r_cmd = reference_signal(t);
    
    % --- PLANT DYNAMICS UPDATE (Based on time t) ---
    m = vp.m; u = vp.u; Iz = vp.Iz; a = vp.a; b = vp.b;
    
    if t >= vp.t_change
        % Reduced stiffness (e.g. icy patch)
        Caf_cur = vp.Ca_f * 0.5;
        Car_cur = vp.Ca_r * 0.5;
    else
        % Nominal stiffness
        Caf_cur = vp.Ca_f;
        Car_cur = vp.Ca_r;
    end
    
    % Recompute Plant Matrices for current time step
    a22 = -(Caf_cur + Car_cur) / (m * u);
    a23 = (Caf_cur + Car_cur) / m;
    a24 = (b * Car_cur - a * Caf_cur) / (m * u);
    a42 = (b * Car_cur - a * Caf_cur) / (Iz * u);
    a43 = (a * Caf_cur - b * Car_cur) / Iz;
    a44 = -(a^2 * Caf_cur + b^2 * Car_cur) / (Iz * u);
    b2 = Caf_cur / m;
    b4 = a * Caf_cur / Iz;
    
    A_p_cur = [0,  1,   0,   0;
               0, a22, a23, a24;
               0,  0,   0,   1;
               0, a42, a43, a44];
    B_p_cur = [0; b2; 0; b4];
    % ----------------------------------------------
    
    % Unpack State
    x_p = X(1:4);
    omega_1 = X(5:7);
    omega_2 = X(8:10);
    x_m = X(11:12);
    phi = X(13:20);
    theta = X(21:28); % [theta1(3); theta2(3); theta3(1); c0(1)]
    
    y_p = x_p(1);
    
    % Reference Model Output
    y_m = C_m * x_m + D_m * r_cmd;
    
    % 1. Construct Regressor Omega
    % omega = [omega_1; omega_2; y_p; r]
    omega = [omega_1; omega_2; y_p; r_cmd];
    
    % 2. Error Calculation
    e1 = y_p - y_m;
    
    % 3. Control Law Calculation (n* = 2) with Normalization
    norm_factor = 1 + phi' * phi;

    % u_p = theta' * omega - (e1 * sgn(c0) * phi' * Gamma * phi) / norm_factor
    correction_term = (e1 * sgn_c0 * (phi' * Gamma * phi)) / norm_factor;
    u_raw = theta' * omega - correction_term;
    
    % No Saturation here!
    u_p = u_raw;
    
    % 4. Adaptation Law with Normalization
    theta_dot = (-Gamma * e1 * phi * sgn_c0) / norm_factor;
    
    % 5. State Derivatives
    
    % Plant (Using updated A_p_cur and B_p_cur)
    dx_p = A_p_cur * x_p + B_p_cur * u_p;
    
    % Filters (omega generation)
    domega_1 = F * omega_1 + g * u_p;
    domega_2 = F * omega_2 + g * y_p;
    
    % Reference Model
    dx_m = A_m * x_m + B_m * r_cmd;
    
    % Filter for phi (n* = 2 specific)
    dphi = -p0 * phi + omega;
    
    dX = [dx_p; domega_1; domega_2; dx_m; dphi; theta_dot];
end

function r = reference_signal(t)
    if t >= 2 && t < 12
        r = 2 + 3 * sqrt(t/12);
    else
        r = 0 + 3 * sqrt(t/12);
    end
end