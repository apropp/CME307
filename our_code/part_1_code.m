clear all; close all;

%% Initialization
rng(5); % set seed
A=rand(2,10); % 10 random points in coordinate plane

%% Q1
% SDP relaxation
[Y_opt_SDP,X_opt_SDP]=SDPsnlsedumi_withoutSteepestDescent(A,3,10,0.3,0,7)

% SOCP relaxation