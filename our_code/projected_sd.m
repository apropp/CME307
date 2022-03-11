clear all; close all; clc; 

%% Initialization

set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 

% rng(2022); % set seed
rng(355)
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = .5; % range within which distances are known
noise_factor = 0; % noise in distance estimation

A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));
M = D <= radius; 

%% Fixed seed experiment for visualization
% rng(355)
% A = rand(2,n_anchors); % n_anchors random points in coordinate plane
% X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
% 
% % D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% % is an anchor and i,j in [6, 15] is a sensor. 
% D = squareform(pdist([A,X]'));
% M = D <= radius; 
% 
% % Set step size; 
% eps = 5*1e-2;
% max_iters = 10000; 
% 
% Z0 = zeros(n_sensors+d);
% Z = rand(size(X));  
% Z0(1:d, 1:d) = eye(d); 
% Z0(d+1:end, 1:d) = Z'; 
% Z0(1:d, d+1:end) = Z; 
% Z0(d+1:end, d+1:end) = Z'*Z; 
% p = 6; 
% 
% [Z0, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, ...
%     n_anchors, X, max_iters, eps, Z0, p);
% visualize_descent(objs, errs2, errsinf, max_iters, 1)
% evaluate_sensors(A, X, Z0, n_sensors, n_anchors, 2)
% 
% %% Varying the eigenvalues
% rng(355)
% A = rand(2,n_anchors); % n_anchors random points in coordinate plane
% X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
% 
% % D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% % is an anchor and i,j in [6, 15] is a sensor. 
% D = squareform(pdist([A,X]'));
% M = D <= radius; 
% 
% % Set step size; 
% eps = 5*1e-2;
% max_iters = 10000; 
% 
% Z0 = zeros(n_sensors+d);
% Z = rand(size(X));  
% Z0(1:d, 1:d) = eye(d); 
% Z0(d+1:end, 1:d) = Z'; 
% Z0(1:d, d+1:end) = Z; 
% Z0(d+1:end, d+1:end) = Z'*Z; 
% ps = 1:n_sensors+d; 
% err2 = zeros(size(p)); 
% errinf = zeros(size(p)); 
% for i = 1:n_sensors+d
%     p = ps(i); 
%     [ZZ, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, ...
%         n_anchors, X, max_iters, eps, Z0, p);
%     err2(i) = norm(ZZ - X); 
%     errinf(i) = norm(ZZ - X, inf); 
% end
% 
% figure(7)
% plot(ps, err2, 'linewidth', 2)
% hold on 
% plot(ps, errinf, 'linewidth', 2)
% hold off 
% xlabel('$p$')
% legend('2-norm error', '$\infty$-norm error')
% 
% 
% %% Varying the step size
% rng(355)
% A = rand(2,n_anchors); % n_anchors random points in coordinate plane
% X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
% 
% % D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% % is an anchor and i,j in [6, 15] is a sensor. 
% D = squareform(pdist([A,X]'));
% M = D <= radius; 
% 
% % Set step size; 
% steps = logspace(-1, -6, 20); 
% max_iters = 10000; 
% 
% Z0 = zeros(n_sensors+d);
% Z = rand(size(X));  
% Z0(1:d, 1:d) = eye(d); 
% Z0(d+1:end, 1:d) = Z'; 
% Z0(1:d, d+1:end) = Z; 
% Z0(d+1:end, d+1:end) = Z'*Z; 
% err2 = zeros(size(steps)); 
% errinf = zeros(size(steps)); 
% p = 6; 
% for i = 1:20
%     step = steps(i); 
%     [ZZ, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, ...
%         n_anchors, X, max_iters, step, Z0, p);
%     err2(i) = norm(ZZ - X); 
%     errinf(i) = norm(ZZ - X, inf); 
% end
% 
% figure(8)
% plot(log10(steps), err2, 'linewidth', 2)
% hold on 
% plot(log10(steps), errinf, 'linewidth', 2)
% hold off 
% xlabel('$\log(\text{step size})$')
% legend('2-norm error', '$\infty$-norm error')

%% 30 random experiments

% Generate data
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));
M = D <= radius; 

% Set step size; 
eps = 1e-2;
max_iters = 10000; 
err2 = zeros(30, 1); 
errinf = zeros(30, 1); 
for q = 1:30
    Z0 = zeros(n_sensors+d);
    Z = rand(size(X));  
    Z0(1:d, 1:d) = eye(d); 
    Z0(d+1:end, 1:d) = Z'; 
    Z0(1:d, d+1:end) = Z; 
    Z0(d+1:end, d+1:end) = Z'*Z; 
    p = 6; 
    [Z0, ~, ~, ~] = proj_solve(A, D, M, d, n_sensors, ...
        n_anchors, X, max_iters, eps, Z0, p);
    err2(q) = norm(Z0 - X); 
    errinf(q) = norm(Z0 - X, inf); 
end

mean(err2)
std(err2)
mean(errinf)
std(errinf)
