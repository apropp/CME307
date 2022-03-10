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

%% Fixed seed experiment 

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

[Z0, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, ...
    n_anchors, X, max_iters, eps, Z0, p);
visualize_descent(objs, errs2, errsinf, max_iters, 1)
evaluate_sensors(A, X, Z0, n_sensors, n_anchors, 2)

%% Fixed seed experiment 

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

[Z0, objs, errs2, errsinf] = proj_solve(A, D, M, d, n_sensors, ...
    n_anchors, X, max_iters, eps, Z0, p);
visualize_descent(objs, errs2, errsinf, max_iters, 1)
evaluate_sensors(A, X, Z0, n_sensors, n_anchors, 2)