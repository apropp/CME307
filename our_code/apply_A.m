
% Helper function to apply OPERATOR A to X.
% Input: 
% - A: anchor locations 
% - M: matrix indicating which i, j pairs for which distance is known
% - d: dimensionality of problem 
% - n_sensors: number of sensors 
% - n_anchors: number of anchors
% - Zhat_k: SGD iteration k 
% Output:
% - AZhat_k: A operator applied to Zhat_k
function [AZhat_k, b] = apply_A(A, M, d, D, n_sensors, n_anchors, Zhat_k)

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
                AZhat_k(t) = sum(sum(A_q.*Zhat_k)); 
                b(t) = D(n_anchors+i, n_anchors+j).^2; 
                t = t+1; 
            end 
        end
        for j = 1:n_anchors
            if M(n_anchors+i, j)
                A_q = [A(:, j); -I(:, i)]*...
                    [A(:, j); -I(:, i)]';
                AZhat_k(t) = sum(sum(A_q.*Zhat_k)); 
                b(t) = D(n_anchors+i, j).^2; 
                t = t+1; 
            end
        end
    end
    % ASSUMES d = 2
    AZhat_k(t) = Zhat_k(1, 1); 
    b(t) = 1; 
    AZhat_k(t+1) = Zhat_k(1, 2); 
    b(t+1) = 0; 
    AZhat_k(t+2) = Zhat_k(2, 1); 
    b(t+2) = 0; 
    AZhat_k(t+3) = Zhat_k(2, 2); 
    b(t+3) = 1; 
%     disp(t+3)
%     disp(ms+ma+d^2)
end