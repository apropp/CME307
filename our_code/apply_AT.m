
% Helper function to apply A^T to X.
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
function Z_k = apply_AT(Zhat_k, p)

end