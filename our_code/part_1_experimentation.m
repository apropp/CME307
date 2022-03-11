clear all; close all; clc; 

%% Initialization
seeds = [771,355,618,1104,979,1026,193,2353,1230];
anchor_numbers = [5, 5, 10];
sensor_numbers = [10, 20, 20];
radii = 0.1:0.1:2;

d = 2; 
radius = 2.5; % range within which distances are known
noise_factor = 0; % noise in distance estimation
eps = 5*1e-3;
max_iters = 300; 

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 

%% Vary sizes, seeds (Adrienne has already saved these figures to add to report appendix)
for i=1:3
    for j=1:3
        t = j+(i-1)*3;
        n_anchors = anchor_numbers(i);
        n_sensors = sensor_numbers(i);
        rng(seeds(t));
        
        A = rand(2,n_anchors); % n_anchors random points in coordinate plane
        X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
        D = squareform(pdist([A,X]'));
        M = D <= radius;
        
        Z0 = rand(d*n_sensors, 1);
        Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
        Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
        [Z_3, objs, errs, errsinf, k] = ...
            nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
        figure
        subplot(1,3,1)
        evaluate_sensors(A, X, Z_1, n_sensors, n_anchors, t);
        subplot(1,3,2)
        evaluate_sensors(A, X, Z_2, n_sensors, n_anchors, t);
        subplot(1,3,3)
        evaluate_sensors(A, X, Z_3, n_sensors, n_anchors, t);
    end
end

%% Table of error, variance for 30 runs
n_anchors = anchor_numbers(1);
n_sensors = sensor_numbers(1);
rng(2022);

nruns = 30;
E2_norms = zeros(5, nruns); % store errors
Inf_norms = zeros(5, nruns); % store errors



for i=1:nruns
    % Generate random data each time
    A = rand(2,n_anchors); % n_anchors random points in coordinate plane
    X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
    D = squareform(pdist([A,X]'));
    M = D <= radius;
    
    % Run optimization problems
    Z0 = rand(d*n_sensors, 1);
    Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
    Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
    Z0_SOCP = reshape(Z_1, [d*n_sensors, 1]);
    Z0_SDP = reshape(Z_2, [d*n_sensors, 1]);
    [Z_3, objs1, errs1, errsinf1, k1] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
    [Z_4, objs2, errs2, errsinf2, k2] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0_SOCP, eps, max_iters);
    [Z_5, objs3, errs3, errsinf3, k3] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0_SDP, eps, max_iters);
    
    E2_norms(1,i) = norm(X-Z_1);
    E2_norms(2,i) = norm(X-Z_2); 
    E2_norms(3,i) = norm(X-Z_3); 
    E2_norms(4,i) = norm(X-Z_4); 
    E2_norms(5,i) = norm(X-Z_5); 
    Inf_norms(1,i) = norm(X-Z_1, inf); 
    Inf_norms(2,i) = norm(X-Z_2, inf); 
    Inf_norms(3,i) = norm(X-Z_3, inf); 
    Inf_norms(4,i) = norm(X-Z_4, inf); 
    Inf_norms(5,i) = norm(X-Z_5, inf); 
end

disp("Means of E2 Norms (SOCP, SDP, NLS)")
mean(E2_norms(1,:))
mean(E2_norms(2,:))
mean(E2_norms(3,:))
mean(E2_norms(4,:))
mean(E2_norms(5,:))
disp("Variances of E2 Norms (SOCP, SDP, NLS)")
var(E2_norms(1,:))
var(E2_norms(2,:))
var(E2_norms(3,:))
var(E2_norms(4,:))
var(E2_norms(5,:))
disp("Means of Inf Norms (SOCP, SDP, NLS)")
mean(Inf_norms(1,:))
mean(Inf_norms(2,:))
mean(Inf_norms(3,:))
mean(Inf_norms(4,:))
mean(Inf_norms(5,:))
disp("Variances of Inf Norms (SOCP, SDP, NLS)")
var(Inf_norms(1,:))
var(Inf_norms(2,:))
var(Inf_norms(3,:))
var(Inf_norms(4,:))
var(Inf_norms(5,:))

%% Vary radii
n_anchors = anchor_numbers(1);
n_sensors = sensor_numbers(1);
rng(seeds(2));

E2_norms = zeros(3, length(radii)); % store errors
Inf_norms = zeros(3, length(radii)); % store errors

rng(seeds(2));
Z0 = rand(d*n_sensors, 1);
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
D = squareform(pdist([A,X]'));

for i=1:length(radii)
    radius = radii(i);
    M = D <= radius;

    Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
    Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
    [Z_3, objs, errs, errsinf, k] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
    
    E2_norms(1,i) = norm(X-Z_1);
    E2_norms(2,i) = norm(X-Z_2); 
    E2_norms(3,i) = norm(X-Z_3); 
    Inf_norms(1,i) = norm(X-Z_1, inf); 
    Inf_norms(2,i) = norm(X-Z_2, inf); 
    Inf_norms(3,i) = norm(X-Z_3, inf); 
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
legend('SOCP', 'SDP', 'NLS')
xlabel('Radius')
ylabel('2-Norm Error')
ylim([0 ymax+1])
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
legend('SOCP', 'SDP', 'NLS')
ylim([0 ymax+1])
xlabel('Radius')
ylabel('Inf-Norm Error')
hold off