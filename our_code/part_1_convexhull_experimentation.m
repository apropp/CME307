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
max_iters = 100; 

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 

% %% Vary sizes, seeds (Adrienne has already saved these figures to add to report appendix)
% for i=1:3
%     for j=1:3
%         t = j+(i-1)*3;
%         n_anchors = anchor_numbers(i);
%         n_sensors = sensor_numbers(i);
%         rng(seeds(t));
%         
%         A = rand(2,n_anchors); % n_anchors random points in coordinate plane
%         X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
%         D = squareform(pdist([A,X]'));
%         M = D <= radius;
%         
%         Z0 = rand(d*n_sensors, 1);
%         Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
%         Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
%         [Z_3, objs, errs] = ...
%             nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
%         figure
%         subplot(1,3,1)
%         evaluate_sensors(A, X, Z_1, n_sensors, n_anchors, t);
%         subplot(1,3,2)
%         evaluate_sensors(A, X, Z_2, n_sensors, n_anchors, t);
%         subplot(1,3,3)
%         evaluate_sensors(A, X, Z_3, n_sensors, n_anchors, t);
%     end
% end

%% Table of error, variance for 30 runs
n_anchors = 5;
n_sensors = 10;
rng(2022);

nruns = 50;
E2_norms = zeros(3, nruns); % store errors
Inf_norms = zeros(3, nruns); % store errors

% Error if pick anchors randomly, put sensors in convex hull randomly
% Error if pick anchors on circle, put sensors in convex hull randomly
% Error if pick anchors in circle, put sensors on outside
% Error if pick anchors and sensors both on outside

for i=1:nruns
    % Generate random data each time
    %A = rand(2,n_anchors); % n_anchors random points in coordinate plane
    %X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

    % Anchors on outside, sensors in convex hull
%     A = (circle_points_outside(n_anchors,0.45,0.5,0.5))';
%     Lam1 = rand(n_anchors, n_sensors);
%     X1 = (A(1,:)*Lam1) ./sum(Lam1);
%     Lam2 = rand(n_anchors, n_sensors);
%     X2 = (A(2,:)*Lam2) ./sum(Lam2);
%     X = [X1; X2];

%     % Anchors on inside, sensors on outside
%     A = circle_points_inside(n_anchors,0.45,0.5,0.5);
%     X = circle_points_outside(n_sensors,0.45,0.5,0.5);
%     A = A';
%     X = X';

    % Anchors on outside, sensors on outside
    A = circle_points_outside(n_anchors,0.45,0.5,0.5);
    X = circle_points_outside(n_sensors,0.45,0.5,0.5);
    A = A';
    X = X';

    D = squareform(pdist([A,X]'));
    M = D <= radius;
    
    % Run optimization problems
    Z0 = rand(d*n_sensors, 1);
    Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
    Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
    [Z_3, objs, errs] = ...
        nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
    
    E2_norms(1,i) = norm(X-Z_1);
    E2_norms(2,i) = norm(X-Z_2); 
    E2_norms(3,i) = norm(X-Z_3); 
    Inf_norms(1,i) = norm(X-Z_1, inf); 
    Inf_norms(2,i) = norm(X-Z_2, inf); 
    Inf_norms(3,i) = norm(X-Z_3, inf); 

    t = i;
    if t < 6
        Z0 = rand(d*n_sensors, 1);
        Z_1 = socp_solve(A, D, M, d, n_sensors, n_anchors);
        Z_2 = sdp_solve(A, D, M, d, n_sensors, n_anchors);
        [Z_3, objs, errs] = ...
            nll_solve(A, D, M, d, n_sensors, n_anchors, X, Z0, eps, max_iters);
        figure
        subplot(2,2,1)
        evaluate_sensors(A, X, Z_1, n_sensors, n_anchors, t);
        title('SOCP')
        axis square
        subplot(2,2,2)
        evaluate_sensors(A, X, Z_2, n_sensors, n_anchors, t);
        title('SDP')
        axis square
        subplot(2,2,3)
        evaluate_sensors(A, X, Z_3, n_sensors, n_anchors, t);
        title('NLL')
        axis square
    end
end

disp("Means of E2 Norms (SOCP, SDP, NLS)")
mean(E2_norms(1,:))
mean(E2_norms(2,:))
mean(E2_norms(3,:))
disp("Variances of E2 Norms (SOCP, SDP, NLS)")
var(E2_norms(1,:))
var(E2_norms(2,:))
var(E2_norms(3,:))
disp("Means of Inf Norms (SOCP, SDP, NLS)")
mean(Inf_norms(1,:))
mean(Inf_norms(2,:))
mean(Inf_norms(3,:))
disp("Variances of Inf Norms (SOCP, SDP, NLS)")
var(Inf_norms(1,:))
var(Inf_norms(2,:))
var(Inf_norms(3,:))

