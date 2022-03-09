
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
function [x1, x2, y] = admm_solve(A, D, M, d, n_sensors, n_anchors, ...
    X, max_iters, eps, x1, x2, y)
    
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
    end
end