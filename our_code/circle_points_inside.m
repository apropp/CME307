% Helper function to generate points inside a circle
% Input: 
% - num: how many points you want to generate
% - R: desired radius of circle
% - x0, y0: center position of circle
% Output: 
% - data: your data points
function [cir_data] = circle_points_inside(num,R,x0,y0)

    angle = 2*pi*rand(num,1);
    radius = R * rand(num,1)* 0.95; % generate points within perimeter
    x = x0 + radius.*cos(angle);
    y = y0 + radius.*sin(angle);

    cir_data = [x,y];
end