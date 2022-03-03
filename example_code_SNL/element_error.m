
% Helper function to run SOCP relaxation solver on given inputs
% Input: 
% - Z: sensor predicted locations 
% - A: anchor locations
% - D: distance matrix 
% - d: dimensionality of sensors/anchors
% - n_sensors: number of sensors
% - n_anchors: number of anchors
% Output:
% - E_s: error matrix on sensors
% - E_a: error matrix on anchors 
function [E_s, E_a] = element_error(Z, A, D, d, n_sensors, n_anchors)
    Z = reshape(Z, [d*n_sensors, 1]); 
    A = reshape(A, [d*n_anchors, 1]); 
    E_s = NaN(n_sensors, n_sensors); 
    E_a = zeros(n_sensors, n_anchors); 
    for i = 1:n_sensors 
        for j = i+1:n_sensors
            E_s(i, j) = (Z(2i-1) - Z(2j-1))^2 + (Z(2i) - Z(2j))^2 - ...
                D(i+n_anchors, j+n_anchors)^2; 
        end
        for j = 1:n_anchors 
            E_a(i, j) = (Z(2i-1) - A(2j-1))^2 + (Z(2i) - A(2j))^2 - ...
                D(i+n_anchors, j); 
        end
    end
end

