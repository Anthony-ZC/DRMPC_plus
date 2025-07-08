close all;
clc;
clear;
%% Parameters
% Unicycle Parameters
params.a = 0.13;
params.rho = 0.0267;
params.b = params.a/params.rho;
params.IC_f = [0.2; 0.2; -pi/2];

% Reference Setting
params.v_r = 0.015;
params.w_r = 0.04;
params.IC_r = [0; 0; pi/3];

% Cost Parameters
params.P = diag([0.4 0.4]);
params.Q = diag([0.2 0.2]);
params.k1 = 1.2;
params.k2 = 1.2;

% Disturbance Parameters
params.A = [0.0024 0.0019]; 
params.f = [2.5 1.25]; 
params.phi = [1 pi/2];
params.m = length(params.A);

% Constraints Parameters
params.lambda_r = sqrt(2)/params.a*params.v_r;
params.epsilon = 0.0593;
params.disturbance_boundary = 0.005;
params.r = (params.a-params.disturbance_boundary)*(1-params.lambda_r)/hypot(params.k1, params.k2);

% Feedback Parameters
params.dob_type = 2;

params.L1 = 15;
params.W = zeros(2*params.m);
for i = 1:params.m
    params.W(2*i-1, 2*i) = params.f(i);
    params.W(2*i, 2*i-1) = -params.f(i);
end
params.C = [1 0 0 1];
params.poles = [-11 -12 -11.5 -13];
params.L2 = place(params.W', params.C',params.poles);

if params.dob_type == 0
    params.epsilon = 0.0593;
elseif params.dob_type == 1
    params.epsilon = 0.0593;
elseif params.dob_type == 2
    params.epsilon = 0.0584;
end

% Simulation Parameters
params.dt = 0.05;
params.T_sim = 15;
params.T_horizon = .4;
params.N = params.T_horizon/params.dt;
params.N_sim = params.T_sim/params.dt;

params.ignore_state_constraints = false;


eig(eye(2*params.m)+params.dt*(params.W-params.L2'*params.C))
%% Holonomic Mobile Robot Dynamic Model
function s_next = holonomic_dynamics(s, u, params)
    B1 = [cos(s(3)) 0;
        sin(s(3)) 0;
        0 1];
    s_dot = B1*u;
    s_next = euler_integration(s, s_dot, params);
end

%% Unicycle Robot Dynamic Model
function s_next = unicycle_dynamics(s, u, d, params)
    B1 = [cos(s(3)) -params.rho*sin(s(3));
        sin(s(3)) params.rho*cos(s(3));
        0 1];
    B2 = [cos(s(3)); sin(s(3)); 0];
    s_dot = B1*u + B2*d;
    
    s_next = euler_integration(s, s_dot, params);
end

%% Tracking Error Dynamic Model
function p_rf_next = tracking_error_dynamics(p_rf, u_f, u_rf, params)  
    p_rf_dot = [0 u_f(2); 
              -u_f(2) 0]*p_rf + u_rf;
    p_rf_next = euler_integration(p_rf, p_rf_dot, params);
end

%% Euler Integration
function s_next = euler_integration(s, v, params)
    s_next = s + v*params.dt;
end

%% Dummy DOB
function d_estimated = dummy_dob(s_f, s_f_last, params)
    B2 = [cos(s_f(3)); sin(s_f(3))];
    d_estimated = pinv(B2,1e-4) * (s_f(1:2)-s_f_last(1:2))/params.dt;
end

%% DOB for Unkown Disturbances
function d_estimated = dob1_observation(z1, s_f, params)
    q1 = params.L1 * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
    d_estimated = z1 + q1;
end

function z1_next = dob1_dynamics(z1, s_f, u_f,d_estimated, params)
    B1 = [cos(s_f(3)) -params.rho*sin(s_f(3));
        sin(s_f(3)) params.rho*cos(s_f(3));
        0 1];
    B2 = [cos(s_f(3)); sin(s_f(3)); 0];
    q1 = params.L1 * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
    l1 = params.L1 * [cos(s_f(3)) sin(s_f(3)) s_f(2)*cos(s_f(3)) - s_f(1)*sin(s_f(3))];

    u_f(1) = u_f(1) - d_estimated;

    z1_dot = -l1*(B2*(q1+z1) + B1*u_f);
    z1_next = euler_integration(z1, z1_dot, params);
end

%% DOB for Known Frequency Disturbances
function d_estimated = dob2_observation(z2, s_f, params)
    q2 = zeros(2 * params.m,1);
    for i =1:2*params.m
        q2(i,:) = params.L2(i) * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
    end
    d_estimated = params.C *(z2 + q2);
end

function z2_next = dob2_dynamics(z2, s_f, u_f,d_estimated, params)
    B1 = [cos(s_f(3)) -params.rho*sin(s_f(3));
        sin(s_f(3)) params.rho*cos(s_f(3));
        0 1];
    B2 = [cos(s_f(3)); sin(s_f(3)); 0];
    q2 = zeros(2 * params.m,1);
    l2 = zeros(2 * params.m,3);
    for i =1:2*params.m
        q2(i,:) = params.L2(i) * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
        l2(i,:) = params.L2(i) * [cos(s_f(3)) sin(s_f(3)) s_f(2)*cos(s_f(3)) - s_f(1)*sin(s_f(3))];
    end

    u_f(1) = u_f(1) - d_estimated;

    z2_dot = (params.W - l2*B2*params.C)*z2 + params.W*q2 - l2*(B2*params.C*q2 + B1*u_f);
    z2_next = euler_integration(z2, z2_dot, params);
end

%% Cost Function
function J = cost_function(u_f, params, s_f, s_r, u_r)
    J = 0;
  
    R = [cos(s_f(3)) sin(s_f(3));
        -sin(s_f(3)) cos(s_f(3))];
    p_rf = R * (s_r(1:2) - s_f(1:2));
    theta_rf = s_r(3) - s_f(3);
    
    for k = 1:params.N
        u_rf = [-u_f(k) + u_r(1)*cos(theta_rf);
            -params.rho*u_f(k + params.N) + u_r(1)*sin(theta_rf)];

        J = J + p_rf'*params.P*p_rf + u_rf'*params.Q*u_rf;
        
        p_rf = tracking_error_dynamics(p_rf, [u_f(k); u_f(k + params.N)], u_rf, params);
        theta_rf_dot = u_r(2) - u_f(k + params.N);
        theta_rf = euler_integration(theta_rf, theta_rf_dot, params);
    end
    J = J + 0.5 * sum(p_rf.^2);
    
end

%% Nonlinear Constraints Function
function [c, ceq] = nonlinear_constraints(u_f, params, s_f, s_r, u_r, d_estimated)
    ceq = 0;
    c = zeros(1, 2*params.N);
    

    R = [cos(s_f(3)) sin(s_f(3));
        -sin(s_f(3)) cos(s_f(3))];
    p_rf = R * (s_r(1:2) - s_f(1:2));
    theta_rf = s_r(3) - s_f(3);

    for k = 1:params.N
        if k>8
            u_f(k) = u_f(8);
            u_f(k + params.N) = u_f(8+ params.N);
        end

        u_rf = [-u_f(k) + u_r(1)*cos(theta_rf);
            -params.rho*u_f(k + params.N) + u_r(1)*sin(theta_rf)];
           
        c(k) = abs(u_f(k)-d_estimated)/params.a + abs(u_f(k + params.N))/params.b -1;

        p_rf = tracking_error_dynamics(p_rf, [u_f(k); u_f(k + params.N)], u_rf, params);
        theta_rf_dot = u_r(2) - u_f(k + params.N);
        theta_rf = euler_integration(theta_rf, theta_rf_dot, params);

        region_radius = params.r*params.N/(k);
        if params.ignore_state_constraints==false
            c(k + params.N) = norm(p_rf)-region_radius;
        end
    end
    if params.ignore_state_constraints==false
        c(end+1) = norm(p_rf)-params.epsilon;
    end
end

%% Disturbance Function
function y = disturbance(t, params)
    y=0;
    for k =1:params.m
        y = y + params.A(k) * sin(params.f(k) * t + params.phi(k)) ;
    end
end

%% DRMPC
options = optimoptions('fmincon');
options.Algorithm='sqp';
options.MaxFunctionEvaluations = 1e5;
options.Display = "off";

s_f = params.IC_f;
s_r = params.IC_r;
u_r = [params.v_r;params.w_r];
s_f_estimated = s_r;

% initialize the disturbance observer
q1 = params.L1 * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
z1 = -q1;

q2 = zeros(2 * params.m,1);
for i =1:2*params.m
    q2(i,:) = params.L2(i) * (s_f(1)*cos(s_f(3)) + s_f(2)*sin(s_f(3)));
end
z2 = -q2;

s_f_last = s_f;

P_r = [];
P_f = [];
P_f_estimated = [];
D_real = [];
D_estimated = [];
Optimal_Control = [];
Compound_Control = [];

operation_time = zeros(params.N_sim+1,1);
%% Plot 2D Visualization
figure;
grid on;
axis equal;
plot(s_r(1), s_r(2), 'r.', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
plot(s_f(1), s_f(2), 'b*', 'MarkerSize', 6, 'LineWidth', 1.5); hold on;
drawnow;

for t = 0:params.N_sim
    
    P_r = [P_r s_r];
    P_f = [P_f s_f];
    P_f_estimated = [P_f_estimated s_f_estimated];
    tic;
    if params.dob_type == 0
        d_estimated = 0;
    elseif params.dob_type == 1
        d_estimated = dob1_observation(z1, s_f, params);
    elseif params.dob_type == 2
        d_estimated = dob2_observation(z2, s_f, params);
    elseif params.dob_type == 3
        d_estimated = dummy_dob(s_f, s_f_last, params);
        s_f_last = s_f;
    end
    
    params.ignore_state_constraints = false;
    u_f0 = [ones(1, params.N)*params.v_r, ones(1, params.N)*params.w_r];
    [u_f_opt, J_opt,exitflag,output]  = fmincon(@(u_f) cost_function(u_f, params, s_f, s_r, u_r), ...
        u_f0, [], [], [], [], [], [], @(u_f) nonlinear_constraints(u_f, params, s_f, s_r, u_r, d_estimated),options);
    
    if exitflag==-2 || exitflag==0
        params.ignore_state_constraints = true;
        [u_f_opt, J_opt,exitflag,output]  = fmincon(@(u_f) cost_function(u_f, params, s_f, s_r, u_r), ...
            u_f0, [], [], [], [], [], [], @(u_f) nonlinear_constraints(u_f, params, s_f, s_r, u_r, d_estimated),options);
    end
    disp(exitflag)
    u_f = [u_f_opt(1); u_f_opt(1 + params.N)];
    
    
    if params.dob_type == 0
        d_estimated = 0;
    elseif params.dob_type == 1
        z1 = dob1_dynamics(z1, s_f, u_f, d_estimated, params);
    elseif params.dob_type == 2
        z2 = dob2_dynamics(z2, s_f, u_f, d_estimated, params);
    elseif params.dob_type == 3
        d_estimated = dummy_dob(s_f, s_f_last, params);
        s_f_last = s_f;
    end

    u_c = [u_f_opt(1)-d_estimated; u_f_opt(1 + params.N)];
    operation_time(t+1)=toc;

    d = disturbance(t*params.dt,params);
    s_f = unicycle_dynamics(s_f, u_c, d, params);
    s_r = holonomic_dynamics(s_r, u_r, params);
    s_f_estimated = unicycle_dynamics(s_f_estimated, u_f, 0 ,params);

    D_real  = [D_real d];
    D_estimated = [D_estimated d_estimated];
    Optimal_Control = [Optimal_Control u_f];
    Compound_Control = [Compound_Control u_c];
    plot(s_r(1), s_r(2), 'r.', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
    plot(s_f(1), s_f(2), 'b*', 'MarkerSize', 6, 'LineWidth', 1.5); hold on;
    drawnow;
end
disp(mean(operation_time))
xlabel('X Position');
ylabel('Y Position');
title('2D Trajectory of P_r and P_f');
hold off;
t = linspace(0, params.T_sim , params.N_sim + 1);
%% Plot Disturbance Estimation Error
figure;
subplot(2,1,1);
plot(t, D_real, 'b', 'LineWidth', 1.5); hold on;
plot(t, D_estimated, 'r-.', 'LineWidth', 1.5);
title('Companion between Real and Estimated Disturbance');
legend('Actual diturbance','Estimated disturbance')
subplot(2,1,2);
plot(t, D_real-D_estimated, 'b', 'LineWidth', 1.5);
title('Disturbance Estimation Error');
%% Plot Control Signal 
figure;
subplot(3,1,1);
plot(t, Optimal_Control(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, Compound_Control(1,:), 'r-.', 'LineWidth', 1.5);
title('Companion between Optimal and Compound Control Signal (Velocity)');
subplot(3,1,2);
plot(t, Optimal_Control(2,:), 'b', 'LineWidth', 1.5);
title('Control Signal (Angular Velocity)');
subplot(3,1,3);
Constraint_list = abs(Compound_Control(1, :))/params.a + abs(Compound_Control(2, :))/params.b;
plot(t, Constraint_list, 'b', 'LineWidth', 1.5);
title('Coupled Constrait Status');
%% Plot Position Error
figure;
Dif = P_r-P_f;
plot(t, Dif(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(t, Dif(2, :), 'r-.', 'LineWidth', 1.5);

P_r_2d = P_r(1:2, :);
P_f_2d = P_f(1:2, :);
title('Position Error');