%% SNL with Noisy Data

clear all; close all; clc; 
% skewed noise - different objective function

%% Initialization
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = 2.5; % range within which distances are known
noise_factor = 0; % noise in distance estimation
eps = 5*1e-3;
max_iters = 300; 

seeds = [771,355,618,1104,979,1026,193,2353,1230];
rng(seeds(2));

Z0 = rand(d*n_sensors, 1); % random initial data for NLS
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
D = squareform(pdist([A,X]'));
M = D <= radius;
        
gamma = .05; 
noise = normrnd(0, gamma, size(D)); 
D_noisy = D + noise; 

%% Noisy SOCP, SDP & NLS

Z_SOCP_noise = noisy_socp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
Z_SDP_noise = noisy_sdp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
[Z_NLS_noise, objs, errs, errsinf, k] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

%% Noisy SOCP & SDP with NLS refinement
Z0 = reshape(Z_SOCP_noise, [d*n_sensors, 1]); 
[Z_SOCP_noise_ref, objs_SOCP_noise_ref, errs_SOCP_noise_ref, errsinf_SOCP_noise_ref, k_SOCP] = ...
    nll_solve(A, D_noisy, M, d, n_sensors, ...
    n_anchors, X, Z0, eps, max_iters);

Z0 = reshape(Z_SDP_noise, [d*n_sensors, 1]); 
[Z_SDP_noise_ref, objs_SDP_noise_ref, errs_SDP_noise_ref, errsinf_SDP_noise_ref, k_SDP] = ...
    nll_solve(A, D_noisy, M, d, n_sensors, ...
    n_anchors, X, Z0, eps, max_iters);

%% Plots

figure
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
subplot(2,3,1)
evaluate_sensors(A, X, Z_SOCP_noise, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
title('(a) SOCP', 'FontSize', 20);
subplot(2,3,2)
evaluate_sensors(A, X, Z_SDP_noise, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
xlabel('$x_1$')
ylabel('$x_2$')
title('(b) SDP', 'FontSize', 20);
subplot(2,3,3)
evaluate_sensors(A, X, Z_NLS_noise, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
xlabel('$x_1$')
ylabel('$x_2$')
title('(c) NLS', 'FontSize', 20);
subplot(2,3,4)
evaluate_sensors(A, X, Z_SOCP_noise_ref, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
title('(d) SOCP w NLS Refinement', 'FontSize', 20);
subplot(2,3,5)
evaluate_sensors(A, X, Z_SDP_noise_ref, n_sensors, n_anchors, 1);
ylim([0 1])
xlim([0 1])
xlabel('$x_1$')
ylabel('$x_2$')
title('(e) SDP w NLS Refinement', 'FontSize', 20);

visualize_descent(objs, errs, errsinf, max_iters, k, 2)
visualize_descent(objs_SOCP_noise_ref, errs_SOCP_noise_ref, errsinf_SOCP_noise_ref, max_iters, k_SOCP, 3)
visualize_descent(objs_SDP_noise_ref, errs_SDP_noise_ref, errsinf_SDP_noise_ref, max_iters, k_SDP, 4)

%% Vary noise level
rng(seeds(1));
Z0 = rand(d*n_sensors, 1);
gammas = 0.01:0.01:0.5;
E2_norms = zeros(5, length(gammas)); % store errors
Inf_norms = zeros(5, length(gammas)); % store errors

for i=1:length(gammas)
    rng(seeds(2));
    gamma = gammas(i);
    noise = normrnd(0, gamma, size(D)); 
    D_noisy = D + noise; 
    
    Z_SOCP = noisy_socp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
    Z_SDP  = noisy_sdp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
    [Z_NLS, objs_NLS, errs_NLS, errsinf_NLS, k_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

    Z0_SOCP = reshape(Z_SOCP, [d*n_sensors, 1]); 
    [Z_SOCP_NLS, objs_SOCP_NLS, errs_SOCP_NLS, errsinf_SOCP_NLS, k_SOCP_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, ...
        n_anchors, X, Z0_SOCP, eps, max_iters);

    Z0_SDP = reshape(Z_SDP, [d*n_sensors, 1]); 
    [Z_SDP_NLS, objs_SDP_NLS, errs_SDP_NLS, errsinf_SDP_NLS, k_SDP_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, ...
        n_anchors, X, Z0_SDP, eps, max_iters);
    
    E2_norms(1,i) = norm(X-Z_SOCP);
    E2_norms(2,i) = norm(X-Z_SDP);
    E2_norms(3,i) = norm(X-Z_NLS);
    E2_norms(4,i) = norm(X-Z_SOCP_NLS);
    E2_norms(5,i) = norm(X-Z_SDP_NLS);
    Inf_norms(1,i) = norm(X-Z_SOCP, inf);
    Inf_norms(2,i) = norm(X-Z_SDP, inf);
    Inf_norms(3,i) = norm(X-Z_NLS, inf);
    Inf_norms(4,i) = norm(X-Z_SOCP_NLS, inf);
    Inf_norms(5,i) = norm(X-Z_SDP_NLS, inf);
end

ymax = max(max(max(Inf_norms)),max(max(E2_norms)))

figure
subplot(1,2,1)
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 

plot(gammas, E2_norms(1,:),'Color',[0 0.4470 0.7410], 'linewidth', 2)
hold on
plot(gammas, E2_norms(2,:),'Color',[0.8500 0.3250 0.0980], 'linewidth', 2)
hold on
plot(gammas, E2_norms(3,:),'Color',[0.3010 0.7450 0.9330], 'linewidth', 2)
hold on
plot(gammas, E2_norms(4,:),'Color',[0.4940, 0.1840, 0.5560], 'linewidth', 2)
hold on
plot(gammas, E2_norms(5,:),'Color',[0.9290, 0.6940, 0.1250], 'linewidth', 2)
legend('SOCP', 'SDP', 'NLS', 'SOCP w NLS', 'SDP w NLS')
xlabel('SD Random Noise')
ylabel('2-Norm Error')
ylim([0 ymax+0.1*ymax])
hold off


subplot(1,2,2)
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
plot(gammas, Inf_norms(1,:),'Color',[0 0.4470 0.7410], 'linewidth', 2)
hold on
plot(gammas, Inf_norms(2,:),'Color',[0.8500 0.3250 0.0980], 'linewidth', 2)
hold on
plot(gammas, Inf_norms(3,:),'Color',[0.3010 0.7450 0.9330], 'linewidth', 2)
hold on
plot(gammas, Inf_norms(4,:),'Color',[0.4940, 0.1840, 0.5560], 'linewidth', 2)
hold on
plot(gammas, Inf_norms(5,:),'Color',[0.9290, 0.6940, 0.1250], 'linewidth', 2)
legend('SOCP', 'SDP', 'NLS', 'SOCP w NLS', 'SDP w NLS')
ylim([0 ymax+0.1*ymax])
xlabel('SD Random Noise')
ylabel('Inf-Norm Error')
hold off

%% Vary radii
radii = 0.1:0.1:2;

E2_norms = zeros(5, length(radii)); % store errors
Inf_norms = zeros(5, length(radii)); % store errors

rng(seeds(1));
Z0 = rand(d*n_sensors, 1);
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
D = squareform(pdist([A,X]'));

gamma = .05; 
noise = normrnd(0, gamma, size(D)); 
D_noisy = D + noise; 

for i=1:length(radii)
    radius = radii(i);
    M = D <= radius;

    Z_SOCP = noisy_socp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
    Z_SDP  = noisy_sdp_solve(A, D_noisy, M, d, n_sensors, n_anchors);
    [Z_NLS, objs_NLS, errs_NLS, errsinf_NLS, k_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

    Z0_SOCP = reshape(Z_SOCP, [d*n_sensors, 1]); 
    [Z_SOCP_NLS, objs_SOCP_NLS, errs_SOCP_NLS, errsinf_SOCP_NLS, k_SOCP_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, ...
        n_anchors, X, Z0_SOCP, eps, max_iters);

    Z0_SDP = reshape(Z_SDP, [d*n_sensors, 1]); 
    [Z_SDP_NLS, objs_SDP_NLS, errs_SDP_NLS, errsinf_SDP_NLS, k_SDP_NLS] = ...
        nll_solve(A, D_noisy, M, d, n_sensors, ...
        n_anchors, X, Z0_SDP, eps, max_iters);

    E2_norms(1,i) = norm(X-Z_SOCP);
    E2_norms(2,i) = norm(X-Z_SDP);
    E2_norms(3,i) = norm(X-Z_NLS);
    E2_norms(4,i) = norm(X-Z_SOCP_NLS);
    E2_norms(5,i) = norm(X-Z_SDP_NLS);
    Inf_norms(1,i) = norm(X-Z_SOCP, inf);
    Inf_norms(2,i) = norm(X-Z_SDP, inf);
    Inf_norms(3,i) = norm(X-Z_NLS, inf);
    Inf_norms(4,i) = norm(X-Z_SOCP_NLS, inf);
    Inf_norms(5,i) = norm(X-Z_SDP_NLS, inf);
end

ymax = max(max(max(Inf_norms)),max(max(E2_norms)))

figure
subplot(1,2,1)
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 

plot(radii, E2_norms(1,:),'Color',[0 0.4470 0.7410], 'linewidth', 2)
hold on
plot(radii, E2_norms(2,:),'Color',[0.8500 0.3250 0.0980], 'linewidth', 2)
hold on
plot(radii, E2_norms(3,:),'Color',[0.3010 0.7450 0.9330], 'linewidth', 2)
hold on
plot(radii, E2_norms(4,:),'Color',[0.4940, 0.1840, 0.5560], 'linewidth', 2)
hold on
plot(radii, E2_norms(5,:),'Color',[0.9290, 0.6940, 0.1250], 'linewidth', 2)
legend('SOCP', 'SDP', 'NLS', 'SOCP w NLS', 'SDP w NLS')
xlabel('Radius')
ylabel('2-Norm Error')
ylim([0 ymax+0.1*ymax])
hold off


subplot(1,2,2)
set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
plot(radii, Inf_norms(1,:),'Color',[0 0.4470 0.7410], 'linewidth', 2)
hold on
plot(radii, Inf_norms(2,:),'Color',[0.8500 0.3250 0.0980], 'linewidth', 2)
hold on
plot(radii, Inf_norms(3,:),'Color',[0.3010 0.7450 0.9330], 'linewidth', 2)
hold on
plot(radii, Inf_norms(4,:),'Color',[0.4940, 0.1840, 0.5560], 'linewidth', 2)
hold on
plot(radii, Inf_norms(5,:),'Color',[0.9290, 0.6940, 0.1250], 'linewidth', 2)
legend('SOCP', 'SDP', 'NLS', 'SOCP w NLS', 'SDP w NLS')
ylim([0 ymax+0.1*ymax])
xlabel('Radius')
ylabel('Inf-Norm Error')
hold off
