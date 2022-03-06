
% Helper function to project Z_k back onto the SDP cone. 
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% - Zhat_k: SGD iteration k 
% - p: number of eigenvalues to use in projection
% Output:
% - Z_k: reconstructed sensor locations projected from Zhat_k using the 
%        p largest eigenvalues
function Z_k = project(Zhat_k, p)
    % Obtain eigendecomposition of Zhat_k
    [V, D] = eigs(Zhat_k, p); 
    lb = zeros(size(D)); 
    Z_k = V * max(D, lb) * V'; 
end

