clear all; close all;

%% Initialization
rng(5); % set seed
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
% n unknown locations = n_sensors - n_anchors
radius = 0.3; % range within which distances are known
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
noise_factor = 0; % noise in distance estimation
degree = 7; %max number of edge distances used per node

D = pdist([A,X]);

%% Q1
% SDP relaxation
%[Y_opt_SDP,X_opt_SDP]=SDPsnlsedumi_withoutSteepestDescent(A,n_anchors,n_sensors,radius,noise_factor,degree)

% SOCP relaxation
n_unknowns = n_sensors - n_anchors;
d = distances(A,n_anchors,n_sensors,radius,noise_factor,degree);
cvx_begin quiet
    variable x(n_unknowns)
    minimize( 0 )
    subject to
    norms(A - x*ones(1,n_unknowns)) <= d;
cvx_end