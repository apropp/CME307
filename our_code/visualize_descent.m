
% Helper function to visualize steepest descent results.
% Input: 
% - objs: objective values during descent
% - max_iters: the number of iterations used
% - fig_id: which figure to use 
function visualize_descent(objs, errs, max_iters, fig_id)

    figure(fig_id)
    
    set(0,'defaultTextInterpreter','latex'); %trying to set the default
    set(0,'defaultAxesFontSize',20)
    set(groot, 'DefaultLegendInterpreter', 'latex')
    set(gcf,'color','w');
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    
    plot(1:max_iters, objs, 'Linewidth', 2)
    hold on 
    plot(1:max_iters, errs, 'Linewidth', 2)
    hold off
    xlabel('iteration')
    legend('objective value', '2-norm error')
end

