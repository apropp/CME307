
% Helper function to visualize steepest descent results.
% Input: 
% - objs: objective values during descent
% - max_iters: the number of iterations used
% - fig_id: which figure to use 
function visualize_descent(objs, errs2, errsinf, max_iters, k, fig_id)

    figure(fig_id)
    
    set(0,'defaultTextInterpreter','latex'); %trying to set the default
    set(0,'defaultAxesFontSize',20)
    set(groot, 'DefaultLegendInterpreter', 'latex')
    set(gcf,'color','w');
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    
    objs = objs(1:k);
    errs2 = errs2(1:k);
    errsinf = errsinf(1:k);
    
    plot(1:k, objs, 'Linewidth', 2)
    hold on 
    plot(1:k, errs2, 'Linewidth', 2)
    hold on 
    plot(1:k, errsinf, 'Linewidth', 2)
    hold off
    xlabel('iteration')
    legend('objective value', '2-norm error', '$\infty$-norm error')
end

