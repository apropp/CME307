
% Helper function to visualize SNL results.
% Input: 
% - A: anchor locations 
% - X: sensor locations 
% - Z: predicted sensor locations
% - (optional) fig_id: which figure, if any, in which to visualize sensors
% Output: 
% - e_2: 2-norm error between sensor locations and reconstruction
% - e_inf: inf-norm error between sensor locations and reconstruction
function [e_2, e_inf] = evaluate_sensors(A, X, Z, n_sensors,...
    n_anchors, fig_id)

	figure(fig_id);
    
    set(0,'defaultTextInterpreter','latex'); %trying to set the default
    set(0,'defaultAxesFontSize',20)
    set(groot, 'DefaultLegendInterpreter', 'latex')
    set(gcf,'color','w');
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    
     % If no visualization parameter is specified, simply compute error. 
     if ~exist('fig_id','var')
        e_2 = norm(X-Z); 
        e_inf = norm(X-Z, inf); 
        return
     end
    hold on;
    % Plot the anchors
    plot(A(1, :), A(2, :), 'd', 'linewidth', 2); 
    % Plot the sensors
    plot(X(1, :), X(2, :), 'og', 'linewidth', 2);
    % Plot the predicted locations
    plot(Z(1, :), Z(2, :), '*r'); 
    for i=1:n_sensors
        plot([Z(1,i)  X(1,i)] , [Z(2,i)  X(2,i)], ...
            'Color', [.5 .5 .5], 'linewidth', 2);
    end
    legend('anchors', 'sensors', 'reconstruction')
    hold off
    xlabel('$x_1$')
    ylabel('$x_2$')
    e_2 = norm(X-Z); 
    e_inf = norm(X-Z, inf); 
end

