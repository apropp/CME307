
% Helper function to run SOCP relaxation solver on given inputs
% Input: 
% - A: anchor locations 
% - D: distance matrix 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of sensors/anchors
% - n_sensors: number of sensors
% - n_anchors: number of anchors
% - Z: predicted sensor locations
% Output:
% - obj: objective value 
function obj = nll_obj(A, D, M, d, n_sensors, n_anchors, Z)
    % Mask for indication of membership in N_x.
    M_s = triu(ones(n_sensors, n_sensors)) - eye(n_sensors); 
    M_s = (M_s & M(n_anchors+1:end, n_anchors+1:end)); 
    % Mask for indication of membership in N_a.
    M_a = M(n_anchors+1:end, 1:n_anchors);
    [E_s, E_a] = element_error(Z, A, D, d, n_sensors, n_anchors); 
    obj = sum(sum((E_s.^2).*M_s + (E_a.^2).*M_a));
end
