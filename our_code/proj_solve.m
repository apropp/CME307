
% Helper function to run projected steepest descent SDP solver on given 
% inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% Output:
% - Z: reconstructed sensor locations
function [Z, objs, errs] = proj_solve(A, D, M, d, n_sensors, n_anchors, ...
    X, max_iters, eps, Z0, p)
    objs = zeros(max_iters, 1); 
    errs = zeros(max_iters, 1); 
    for k = 1:max_iters
        % Step
        [AZ0, b] = apply_A(A, M, d, D, n_sensors, n_anchors, Z0);
        grad = apply_AT(A, M, d, n_sensors, n_anchors, AZ0-b); 
        Zhat_0 = Z0 - eps*grad; 
        Z0 = project(Zhat_0, p); 
        % Compute error
        [AZ0, b] = apply_A(A, M, d, D, n_sensors, n_anchors, Z0);
        objs(k) = 1/2*(norm(AZ0-b)).^2; 
        errs(k) = norm(X - Z0(1:d, d+1:end)); 
    end
    Z = Z0(1:d, d+1:end);
end

