clear all; close all; clc; 

%% Initialization
rng(2022); % set seed
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = 0.3; % range within which distances are known
noise_factor = 0; % noise in distance estimation

% TODO - do we need this? 
% degree = 7; %max number of edge distances used per node 

A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));

%% Q1

% TODO - fix this:
% Dummy variable M to keep track of indices we want to add constraint for.
M = eye(3);

% Example usage... 
Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
% TODO - nonlinear least squares 
% Z_3 = nll_solve(A, D, M, d, n_sensors, n_anchors);

evaluate_sensors(A, X, Z_1, d, n_sensors, n_anchors, 1)
evaluate_sensors(A, X, Z_2, n_sensors, n_anchors)

