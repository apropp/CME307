
% Helper function to run SOCP relaxation solver on given inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% Output:
% - Z: reconstructed sensor locations
function Z = socp_solve(A, D, M, d, n_sensors, n_anchors)
    % SOCP relaxation code 
    % TODO - remove constraints for distant nodes
    cvx_begin quiet
        variable Z(d, n_sensors)
        minimize( 0 )
            subject to
            for i=1:n_sensors
                sum_square_abs(A - Z(:, i)*ones(1, n_anchors)) <= ...
                    D(n_anchors+i, 1:n_anchors).^2;
                sum_square_abs(Z - Z(:, i)*ones(1, n_sensors)) <= ...
                    D(n_anchors+i, n_anchors+1:end).^2;
            end
    cvx_end
end

