clear all; close all; clc; 

%% Initialization

% rng(2022); % set seed
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = 2.5; % range within which distances are known
noise_factor = 0; % noise in distance estimation

A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));
M = D <= radius; 

%% Projected steepest descents 
% Set step size; 
eps = 1e-2;
max_iters = 3000; 

Z0 = zeros(n_sensors+d);
rng(771)
Z = rand(size(X));  
Z0(1:d, 1:d) = eye(d); 
Z0(d+1:end, 1:d) = Z'; 
Z0(1:d, d+1:end) = Z; 
Z0(d+1:end, d+1:end) = Z'*Z; 
p = 6; 

[Z0, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, n_anchors, ...
    X, max_iters, eps, Z0, p);
visualize_descent(objs, errs2, errsinf, max_iters, 1)
evaluate_sensors(A, X, Z0, n_sensors, n_anchors, 2)


%% Scratch

% Vanilla SOCP
% Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
% Vanilla SDP
% Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
% Vanilla SGD (NLL)
% eps = 5*1e-3;
% max_iters = 100; 
% Z0 = rand(d*n_sensors, 1);
% [Z_3, objs, errs] = ...
%     nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

%% Q2
% eps = 5*1e-3;
% max_iters = 100; 
% Z0 = reshape(Z_1, [d*n_sensors, 1]); 
% [Z_1_ref, objs_3, errs_3] = ...
%     nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

%% Q3 and Q4
% gamma = .05; 
% noise= normrnd(0, gamma, size(D)); 
% D_noisy = D + noise; 
% Z_2_noise = noisy_sdp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
% Z_3_noise = noisy_socp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
% TODO SGD noisy???? ask yinyu
% 
% eps = 5*1e-3;
% max_iters = 100; 
% Z0 = reshape(Z_2_noise, [d*n_sensors, 1]); 
% [Z_2_noise_ref, objs_2_noise_ref, errs_2_noise_ref] = ...
%     nll_solve(A, D_noisy, M, d, n_sensors, ...
%     n_anchors, X, Z0, eps, max_iters);
% 
% eps = 5*1e-3;
% max_iters = 100; 
% Z0 = reshape(Z_3_noise, [d*n_sensors, 1]); 
% [Z_3_noise_ref, objs_3_noise_ref, errs_3_noise_ref] = ...
%     nll_solve(A, D_noisy, M, d, n_sensors, ...
%     n_anchors, X, Z0, eps, max_iters);

%% Plots 

% % Vanilla SOCP 
% evaluate_sensors(A, X, Z_1, n_sensors, n_anchors, 1)
% % Vanilla SDP 
% evaluate_sensors(A, X, Z_2, n_sensors, n_anchors, 2)
% % Vanilla SGD (NLL) 
% evaluate_sensors(A, X, Z_3, n_sensors, n_anchors, 3)
% visualize_descent(objs_3, errs_3, max_iters, 4)
% 
% % Vanilla SOCP + SGD refinement (NLL) 
% evaluate_sensors(A, X, Z_1_ref, n_sensors, n_anchors, 3)
% visualize_descent(objs_3, errs_3, max_iters, 4)
% 
% % Noisy SOCP
% evaluate_sensors(A, X, Z_2_noise, n_sensors, n_anchors, 5)
% % Noisy SDP
% evaluate_sensors(A, X, Z_3_noise, n_sensors, n_anchors, 6)
% 
% % Noisy SOCP + SGD refinement
% evaluate_sensors(A, X, Z_2_noise_ref, n_sensors, n_anchors, 7)
% visualize_descent(objs_2_noise_ref, errs_2_noise_ref, max_iters, 4)
% % Noisy SDP + SGD refinement
% evaluate_sensors(A, X, Z_3_noise_ref, n_sensors, n_anchors, 8)
% visualize_descent(objs_3_noise_ref, errs_3_noise_ref, max_iters, 4)



