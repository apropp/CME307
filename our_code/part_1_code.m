clear all; close all;

%% Initialization
rng(5); % set seed
n_anchors = 3; % known locations
n_sensors = 10; % total points
% n unknown locations = n_sensors - n_anchors
radius = 0.3; % range within which distances are known
A=rand(2,10); % 10 random points in coordinate plane
noise_factor = 0; % noise in distance estimation
degree = 7; %max number of edge distances used per node

%% Q1
% SDP relaxation
[Y_opt_SDP,X_opt_SDP]=SDPsnlsedumi_withoutSteepestDescent(A,n_anchors,n_sensors,radius,noise_factor,degree)

% SOCP relaxation
n_unknowns = n_sensors - n_anchors;
d = distances(A,n_anchors,n_sensors,radius,noise_factor,degree);
cvx_begin quiet
    variable x(n_unknowns)
    minimize( 0 )
    subject to
    norms(A - x*ones(1,n_unknowns)) <= d;
cvx_end