
% Helper function to run SOCP relaxation solver on given inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% - Z0: initial guess
% - eps: learning rate 
% - max_iters: maximum number of iterations to use
% Output:
% - Z: reconstructed sensor locations
% - objs: objective values as a function of k 
% - errs2: 2-norm errors as a function of k 
% - errsinf: inf-norm errors as a function of k 
function [Z, objs, errs2, errsinf, k] = nll_solve(A, D, M, d, n_sensors, n_anchors, X, ...
    Z0, eps, max_iters)
    objs = zeros(max_iters, 1); 
    errs2 = zeros(max_iters, 1); 
    errsinf = zeros(max_iters, 1); 
    for k = 1:max_iters
        Z0 = Z0 - eps*nll_obj_grad(A, D, M, d, n_sensors, n_anchors, Z0); 
        objs(k) = nll_obj(A, D, M, d, n_sensors, n_anchors, Z0); 
        errs2(k) = norm(reshape(X, [d*n_sensors, 1]) - Z0); 
        errsinf(k) = norm(reshape(X, [d*n_sensors, 1]) - Z0, inf); 
        % Convergence criteria
        if k>1 && (abs(objs(k) - objs(k-1)) < 1e-5)
            break
        end
    end
    Z = zeros(2, n_sensors); 
    for i = 1:n_sensors
        Z(:, i) = [Z0(2*i-1); Z0(2*i)]';
    end
end
