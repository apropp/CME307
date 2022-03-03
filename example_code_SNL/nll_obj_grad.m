
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
function grad = nll_obj_grad(A, D, M, d, n_sensors, n_anchors, Z)
    % Mask for indication of membership in N_x.
    M_s = triu(ones(n_sensors, n_sensors)) - eye(n_sensors); 
    M_s = (M_s & M(n_anchors+1:end, n_anchors+1:end)); 
    % Mask for indication of membership in N_a.
    M_a = M(n_anchors+1:end, 1:n_anchors);
    
    Z = reshape(Z, [d*n_sensors, 1]); 
    A = reshape(A, [d*n_anchors, 1]); 
    grad = zeros(2*n_sensors, 1); 
    [E_s, E_a] = element_error(Z, A, D, d, n_sensors, n_anchors); 
    
    W_s = NaN(2*n_sensors, 2*n_sensors); 
    W_a = NaN(2*n_sensors, 2*n_anchors); 
    
    for i = 1:n_sensors
        for j = 1:n_sensors
            if M_s(i, j) 
                W_s(2*i-1, 2*j-1) = Z(2*i-1) - Z(2*j-1);
                W_s(2*i, 2*j) = Z(2*i) - Z(2*j); 
            else
                W_s(2*i-1, 2*j-1) = 0;
                W_s(2*i, 2*j) = 0; 
            end
        end
        for j = 1:n_anchors
            if M_a(i, j)
                W_a(2*i-1, 2*j-1) = Z(2*i-1) - A(2*j-1);
                W_a(2*i, 2*j) = Z(2*i) - A(2*j); 
            else
                W_a(2*i-1, 2*j-1) = 0;
                W_a(2*i, 2*j) = 0; 
            end
        end
    end
    
    for t = 1:n_sensors
        grad(2*t-1) = 4*sum(E_s(t,:).* W_s(2*t-1,1:2:end)) -...
            4*sum(E_s(:,t).*W_s(1:2:end,2*t-1))+...
            4*sum(E_a(t,:).*W_a(2*t-1,1:2:end));
        grad(2*t) = 4*sum(E_s(t,:).* W_s(2*t,2:2:end)) -...
            4*sum(E_s(:,t).*W_s(2:2:end,2*t))+...
            4*sum(E_a(t,:).*W_a(2*t,2:2:end));
    end
end
