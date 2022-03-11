%% SNL with Noisy Data

clear all; close all; clc; 
% skewed noise - different objective function

%% Initialization
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = 2.5; % range within which distances are known
noise_factor = 0; % noise in distance estimation

seeds = [771,355,618,1104,979,1026,193,2353,1230];
rng(seeds(2));

A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
D = squareform(pdist([A,X]'));
M = D <= radius;
        
gamma = .05; 
noise = normrnd(0, gamma, size(D)); 
D_noisy = D + noise; 

%% Noisy SOCP & SDP

Z_2_noise = noisy_socp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
Z_3_noise = noisy_sdp_solve(A, D_noisy, M, d, n_sensors, n_anchors);


%% Noisy SOCP & SDP with NLS refinement
eps = 5*1e-3;
max_iters = 300; 

Z0 = reshape(Z_2_noise, [d*n_sensors, 1]); 
[Z_2_noise_ref, objs_2_noise_ref, errs_2_noise_ref, errsinf_2_noise_ref, k2] = ...
    nll_solve(A, D_noisy, M, d, n_sensors, ...
    n_anchors, X, Z0, eps, max_iters);

Z0 = reshape(Z_3_noise, [d*n_sensors, 1]); 
[Z_3_noise_ref, objs_3_noise_ref, errs_3_noise_ref, errsinf_3_noise_ref, k3] = ...
    nll_solve(A, D_noisy, M, d, n_sensors, ...
    n_anchors, X, Z0, eps, max_iters);

%% Plots

figure
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
subplot(2,2,1)
evaluate_sensors(A, X, Z_2_noise, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
title('(a) SOCP', 'FontSize', 22);
subplot(2,2,2)
evaluate_sensors(A, X, Z_3_noise, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
xlabel('$x_1$')
ylabel('$x_2$')
title('(b) SDP', 'FontSize', 22);
subplot(2,2,3)
evaluate_sensors(A, X, Z_2_noise_ref, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
title('(c) SOCP w NLS Refinement', 'FontSize', 22);
subplot(2,2,4)
evaluate_sensors(A, X, Z_3_noise_ref, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
xlabel('$x_1$')
ylabel('$x_2$')
title('(d) SDP w NLS Refinement', 'FontSize', 22);

visualize_descent(objs_2_noise_ref, errs_2_noise_ref, errsinf_2_noise_ref, max_iters, k2, 3)
visualize_descent(objs_3_noise_ref, errs_3_noise_ref, errsinf_3_noise_ref, max_iters, k3, 4)

