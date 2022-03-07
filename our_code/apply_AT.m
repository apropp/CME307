
% Helper function to apply OPERATOR A to X.
% Input: 
% - A: anchor locations 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% - Zhat_k: SGD iteration k 
% Output:
% - A^Ty: A^T operator applied to Zhat_k
function [ATy] = apply_AT(A, M, d, n_sensors, n_anchors, y)

    ATy = zeros(n_sensors+d); 

    I = eye(n_sensors); 
    ms = sum(sum(triu(M(n_anchors+1:end, n_anchors+1:end)) - ...
        eye(n_sensors))); 
    ma = sum(sum(M(n_anchors+1:end, 1:n_anchors))); 
    AZhat_k = zeros(ms+ma+d^2, 1);
    b = zeros(ms+ma+d^2, 1); 
    
    t = 1; 
    for i = 1:n_sensors
        for j = i+1:n_sensors 
            if M(n_anchors+i, n_anchors+j) 
                A_q = [zeros(d, 1); I(:, i) - I(:, j)]*...
                    [zeros(d, 1); I(:, i) - I(:, j)]';
                ATy = ATy + y(t)*A_q; 
                t = t+1; 
            end 
        end
        for j = 1:n_anchors
            if M(n_anchors+i, j)
                A_q = [A(:, j); -I(:, i)]*...
                    [A(:, j); -I(:, i)]';
                ATy = ATy + y(t)*A_q; 
                t = t+1; 
            end
        end
    end
    
    % ASSUMES d = 2
    Ae1 = zeros(size(ATy)); 
    Ae1(1, 1) = 1; 
    ATy = ATy + y(t) * Ae1; 
    
    Ae2 = zeros(size(ATy)); 
    Ae2(1, 2) = 1; 
    ATy = ATy + y(t+1) * Ae2; 
    
    Ae3 = zeros(size(ATy)); 
    Ae3(2, 1) = 1; 
    ATy = ATy + y(t+2) * Ae3; 
        
    Ae4 = zeros(size(ATy)); 
    Ae4(2, 2) = 1; 
    ATy = ATy + y(t+3) * Ae4; 
    
%     disp(t+3)
%     disp(ms+ma+d^2)
end