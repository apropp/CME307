
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
function [x1, x2, y, objs, errs2, errsinf, finalobj, final2, finalinf] = admm_solve(A, D, M, d, ...
    n_sensors, n_anchors, X, max_iters, eps, x1, x2, y)

    tic
    objs = zeros(max_iters, 1); 
    errs2 = zeros(max_iters, 1); 
    errsinf = zeros(max_iters, 1); 
    
    for k = 1:max_iters
        % Optimize for x1
        cvx_begin quiet
            variable x1(d, n_sensors)
            f = 0; 
            for i = 1:n_sensors
                for j = i+1:n_sensors
                    if M(n_anchors+i, n_anchors+j)
                        f = f + power(((x1(:, i) - x1(:, j))'*...
                            (x2(:, i) - x2(:, j)) - ...
                            D(n_anchors+i, n_anchors+j)^2), 2);
                    end
                end
                for j = 1:n_anchors
                    if M(n_anchors+i, j)
                        f = f + power(((x1(:, i) - A(:, j))'*...
                            (x2(:, i) - A(:, j)) - ...
                            D(n_anchors+i, j)^2), 2);
                    end
                end
                f = f - y(:, i)'*(x1(:, i) - x2(:, i)); 
                f = f + eps/2 * ...
                    (x1(:, i) - x2(:, i))'*(x1(:, i) - x2(:, i));
            end
            minimize( f )
        cvx_end
        
        % Optimize for x2
        cvx_begin quiet
            variable x2(d, n_sensors)
            f = 0; 
            for i = 1:n_sensors
                for j = i+1:n_sensors
                    if M(n_anchors+i, n_anchors+j)
                        f = f + power(((x1(:, i) - x1(:, j))'*...
                            (x2(:, i) - x2(:, j)) - ...
                            D(n_anchors+i, n_anchors+j)^2), 2);
                    end
                end
                for j = 1:n_anchors
                    if M(n_anchors+i, j)
                        f = f + power(((x1(:, i) - A(:, j))'*...
                            (x2(:, i) - A(:, j)) - ...
                            D(n_anchors+i, j)^2), 2);
                    end
                end
                f = f - y(:, i)'*(x1(:, i) - x2(:, i)); 
                f = f + eps/2 * ...
                    (x1(:, i) - x2(:, i))'*(x1(:, i) - x2(:, i));
            end
            minimize( f )
        cvx_end
        
        % Update y 
        y = y - eps*(x1 - x2); 

        
        
        % Set current prediction to be the mean of x1 and x2. 
        X_pred = (x1 + x2)/2; 
        objs(k) = nll_obj(A, D, M, d, n_sensors, n_anchors, X_pred); 
        errs2(k) = norm(reshape(X, [d*n_sensors, 1]) - reshape(X_pred, [d*n_sensors, 1])); 
        errsinf(k) = norm(reshape(X, [d*n_sensors, 1]) - reshape(X_pred, [d*n_sensors, 1]), inf);
        
        % Convergence criteria
        if k>1 && (abs(objs(k) - objs(k-1)) < 1e-8)
            finalobj = objs(k);
            final2 = errs2(k);
            finalinf = errsinf(k);
            break
        end
    end
    if k == max_iters
        finalobj = objs(k);
        final2 = errs2(k);
        finalinf = errsinf(k);
    end
    toc
end