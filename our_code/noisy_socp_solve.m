
% Helper function to run SOCP relaxation solver on given inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% Output:
% - Z: reconstructed sensor locations
function Z = noisy_socp_solve(A, D, M, d, n_sensors, n_anchors)
    % SOCP relaxation code 
    % TODO - remove constraints for distant nodes
    M_s = triu(ones(n_sensors, n_sensors)) - eye(n_sensors); 
    M_s = (M_s & M(n_anchors+1:end, n_anchors+1:end)); 
    cvx_begin quiet
        variable Z(d, n_sensors)
        variable D_s(n_sensors, n_sensors)
        variable D_a(n_sensors, n_anchors)
        minimize( sum(sum(D_a.*M(1:n_sensors, 1:n_anchors))) + ...
            sum(sum(D_s.*M_s)) )
            subject to
                D_s >= 0; 
                D_a >= 0;  
                for i = 1:n_sensors 
                    for j = i+1:n_sensors
                        if M(n_anchors+i, n_anchors+j) > 0
                            norm(Z(:, i) - Z(:, j)) - D_s(i, j) <= ...
                                D(n_anchors+i, n_anchors+j);
                        end
                    end
                end
                for i=1:n_sensors
                    for j=1:n_anchors
                        if M(n_anchors+i, j) > 0
                            norm(Z(:, i) - A(:, j)) - D_a(i, j) <= ...
                                D(n_anchors+i, j);
                        end
                    end
                end
    cvx_end
end

