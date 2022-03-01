
% Helper function to run SDP relaxation solver on given inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% Output:
% - Z: reconstructed sensor locations
function Z = sdp_solve(A, D, M, d, n_sensors, n_anchors)
    m = d + n_sensors; 
    Id = eye(m - d); 
    % SDP relaxation
    cvx_begin quiet
        variable Z(m, m) semidefinite
        minimize( 0 )
        subject to
            Z(1:d, 1:d) == eye(d); 
            for i=1:n_sensors
                for j = i+1:n_sensors
                    (Id(:, i) - Id(:, j))'*Z(d+1:end, d+1:end)*...
                        (Id(:, i) - Id(:, j)) ...
                        == D(n_anchors+i, n_anchors+j)^2;
                end
            end
            for k = 1:n_anchors
                for j = 1:n_sensors
                    [A(:, k); -Id(:, j)]'*Z*[A(:, k); -Id(:, j)] ...
                        == D(k, n_anchors+j)^2;
                end
            end
    cvx_end
    Z = Z(1:d, d+1:end);
end

