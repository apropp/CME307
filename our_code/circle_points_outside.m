% Helper function to generate points on the approximate perimeter of a circle
% Input: 
% - num: how many points you want to generate
% - R: desired radius of circle
% - x0, y0: center position of circle
% Output: 
% - data: your data points
function [cir_data] = circle_points_outside(num,R,x0,y0)

    angle = 2*pi*rand(num,1);
    radius = 0.9 * R + 0.2 * R * rand(num,1); % generate points around perimeter
    x = x0 + radius.*cos(angle);
    y = y0 + radius.*sin(angle);

    cir_data = [x,y];
end