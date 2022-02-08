% start from the given feasible initial solution x
% form the gradient vector
g=[x(1)-1;x(2)-1];
A=[1 1];
% compute the multipliers
y=(A*A')\(A*g)
% form the gradient-projection
gp=g-A'*y;
% using fixed step-size 0.9
x=x-0.9*gp
% One Step of Gradient-projection method for solving the class example
% (slide 13 of Lecture Note 12)
%      minimize    0.5(x(1)-1)^2+0.5(x(2)-1)^2
%      subject to    x(1)+x(2)-1=0
%
%      Input: any initial feasible x
%