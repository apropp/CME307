clear all; close all; clc; 

%% Initialization

rng(2022); % set seed
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

%% Q1

% Example usage... 
% Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
% Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);

% TODO - nonlinear least squares 
eps = 1e-3;
max_iters = 200; 
Z0 = rand(d*n_sensors, 1);
[Z_3, objs, errs] = ...
    nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);

% Plots 
% evaluate_sensors(A, X, Z_1, n_sensors, n_anchors, 1)
% evaluate_sensors(A, X, Z_2, n_sensors, n_anchors, 2)
evaluate_sensors(A, X, Z_3, n_sensors, n_anchors, 3)
visualize_descent(objs, errs, max_iters, 4)

