clear all; close all; clc; 

%% Initialization

rng(2022); % set seed
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

%% Q1 and Q2

% % Vanilla SOCP
% Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
% % Vanilla SDP
% Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
% % Vanilla SGD (NLL)
% eps = 5*1e-3;
% max_iters = 100; 
% Z0 = rand(d*n_sensors, 1);
% [Z_3, objs, errs] = ...
%     nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

%% Part 3
eps = 10;
max_iters = 5; 


Z0 = zeros(n_sensors+d);
gamma = .05; 
noise = normrnd(0, gamma, size(X)); 

% Z = X + noise; 
% Z = rand(size(X)); 
% Z0(1:d, 1:d) = eye(d); 
% Z0(d+1:end, 1:d) = Z'; 
% Z0(1:d, d+1:end) = Z; 
% Z0(d+1:end, d+1:end) = Z'*Z; 
% p = 6; 

% gamma = .05; 
% noise= normrnd(0, gamma, size(Z0)); 
% Z0 = Z0 + noise; 

% [Z0, objs, errs] = proj_solve(A, D, M, d, n_sensors, n_anchors, ...
%     X, max_iters, eps, Z0, p);
% visualize_descent(objs, errs, max_iters, 1)
% evaluate_sensors(A, X, Z0, n_sensors, n_anchors, 2)

x1 = rand(d, n_sensors); 
x2 = rand(d, n_sensors); 
y = rand(d, n_sensors); 

[x1, x2, y] = admm_solve(A, D, M, d, n_sensors, n_anchors, ...
    X, max_iters, eps, x1, x2, y)
