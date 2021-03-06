
% Helper function to run SDP relaxation solver on given inputs.
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% Output:
% - Z: reconstructed sensor locations
function Z = noisy_sdp_solve(A, D, M, d, n_sensors, n_anchors)
    m = d + n_sensors; 
    Id = eye(m - d); 
    M_s = triu(ones(n_sensors, n_sensors)) - eye(n_sensors); 
    M_s = (M_s & M(n_anchors+1:end, n_anchors+1:end)); 
    % SDP relaxation
    cvx_begin quiet
        variable Z(m, m) semidefinite
        variable D_s(n_sensors, n_sensors)
        variable D_sp(n_sensors, n_sensors)
        variable D_a(n_sensors, n_anchors)
        variable D_ap(n_sensors, n_anchors)
        minimize( sum(sum((D_a + D_ap).*M(1:n_sensors, 1:n_anchors))) ...
            + sum(sum((D_s + D_sp).*M_s)) )
        subject to
            Z(1:d, 1:d) == eye(d); 
            D_s >= 0; 
            D_sp >= 0; 
            D_a >= 0; 
            D_ap >= 0; 
            for i=1:n_sensors
                for j = i+1:n_sensors
                    if M(n_anchors+i, n_anchors+j)
                        (Id(:, i) - Id(:, j))'*Z(d+1:end, d+1:end)*...
                            (Id(:, i) - Id(:, j)) + ...
                            D_s(i, j) - D_sp(i, j) ...
                            == D(n_anchors+i, n_anchors+j)^2;
                    end
                end
            end
            for k = 1:n_anchors
                for j = 1:n_sensors
                    if M(k, n_anchors+j) 
                        [A(:, k); -Id(:, j)]'*Z*[A(:, k); -Id(:, j)] + ...
                            D_a(j, k) - D_ap(j, k) ... 
                            == D(k, n_anchors+j)^2;
                    end
                end
            end
    cvx_end
    Z = Z(1:d, d+1:end);
end

