clear all; close all; clc; 

%% Initialization
seeds = [771,355,618,1104,979,1026,193,2353,1230];

rng(355); % set seed
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
radius = 0.5; % range within which distances are known
noise_factor = 0; % noise in distance estimation

A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));
M = D <= radius; 

eps_list = [-3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3];
max_iters = 250; 

% For Testing
%eps_list = [1,2];
%max_iters = 2;

x1_0 = rand(d, n_sensors); 
x2_0 = rand(d, n_sensors); 
y_0 = rand(d, n_sensors); 

objs_list = zeros([1,length(eps_list)]);
err2_list = zeros([1,length(eps_list)]);
errinf_list = zeros([1,length(eps_list)]);

for i=1:length(eps_list)
    x1 = x1_0;
    x2 = x2_0;
    y = y_0;

    eps = 10^(eps_list(i));

    [x1, x2, y, objs, errs2, errsinf, finalobj, final2, finalinf] = admm_solve(A, D, M, d, ...
        n_sensors, n_anchors, X, max_iters, eps, x1, x2, y);
    
    objs_list(i) = finalobj;
    err2_list(i) = final2;
    errinf_list(i) = finalinf;
end


figure(1)

set(0,'defaultTextInterpreter','latex'); %trying to set the default
set(0,'defaultAxesFontSize',20)
set(groot, 'DefaultLegendInterpreter', 'latex')
set(gcf,'color','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 

plot(eps_list, log10(err2_list), 'Linewidth', 2)
hold on 
plot(eps_list, log10(errinf_list), 'Linewidth', 2)
hold off
xlabel('$log_{10}(\beta)$')
legend('2-norm error', '$\infty$-norm error')
saveas(gcf,"ADMMbeta.png")

writematrix(objs_list)
writematrix(err2_list)
writematrix(errinf_list)


% %%% Multiple Runs with Multiple Seeds
% 
% d = 2; 
% n_anchors = 5; % known locations
% n_sensors = 10; % number of sensors
% radius = 0.5; % range within which distances are known
% noise_factor = 0; % noise in distance estimation
% eps = 1;
% max_iters = 500;
% rng(355);
% 
% nruns = 15; % 15
% E2_norms = zeros(1, nruns); % store errors
% Inf_norms = zeros(1, nruns); % store errors
% fig_count = 1;
% 
% for i=1:nruns
%     disp(i)
%     A = rand(2,n_anchors); % n_anchors random points in coordinate plane
%     X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
% 
%     % D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
%     % is an anchor and i,j in [6, 15] is a sensor. 
%     D = squareform(pdist([A,X]'));
%     M = D <= radius; 
% 
%     x1_0 = rand(d, n_sensors); 
%     x2_0 = rand(d, n_sensors); 
%     y_0 = rand(d, n_sensors); 
% 
%     [x1, x2, y, objs, errs2, errsinf, finalobj, final2, finalinf] = admm_solve_rando(A, D, M, d, ...
%         n_sensors, n_anchors, X, max_iters, eps, x1_0, x2_0, y_0);
%     Z = (x1+x2)/2;
% 
%     E2_norms(1,i) = final2;
%     Inf_norms(1,i) = finalinf;
% 
%     visualize_descent(objs, errs2, errsinf, max_iters,max_iters,fig_count)
%     evaluate_sensors(A, X, Z, n_sensors, n_anchors, fig_count+1)
%     fig_count = fig_count+2;
%     
% end
% 
% disp("Means of E2 Norms (ADMM)")
% mean(E2_norms(1,:))
% disp("Variances of E2 Norms (ADMM)")
% var(E2_norms(1,:))
% disp("Means of Inf Norms (ADMM)")
% mean(Inf_norms(1,:))
% disp("Variances of Inf Norms (ADMM)")
% var(Inf_norms(1,:))

