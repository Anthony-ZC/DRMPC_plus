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

busInfo = Simulink.Bus.createObject(params);
bus = evalin('base', busInfo.busName);


q1 = params.L1 * (params.IC_f(1)*cos(params.IC_f(3)) + params.IC_f(2)*sin(params.IC_f(3)));
z1 = -q1;

q2 = zeros(2 * params.m,1);
for i =1:2*params.m
    q2(i,:) = params.L2(i) * (params.IC_f(1)*cos(params.IC_f(3)) + params.IC_f(2)*sin(params.IC_f(3)));
end
z2 = -q2;
