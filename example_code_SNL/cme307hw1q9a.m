clear all; close all;
%% Initialization
A = [1,-1,0;0,0,2];
%%% 1) random initialization choice (remove comment to enable)
% x1 = randn(2, 1);
% x2 = randn(2, 1);
%%% 2) random initialization choice inside the conv-hull (remove comment to
%%% enable)
% lambda1 = rand(3,1);
% x1 = A * lambda1 / sum(lambda1);
% lambda2 = rand(3,1);
% x2 = A * lambda2 / sum(lambda2);
%%% 2) special initialization choice (add comment to disable)
lambda1 = rand(3,1);
x1 = A * lambda1 / sum(lambda1);
lambda2 = rand(3,1);
x2 = [x1, A(:,2:3)] * lambda2 / sum(lambda2);
%% Plot the figure
scatter(A(1,:),A(2,:), 'k', 'filled');
hold on;
scatter(x1(1), x1(2), 'r*');
scatter(x2(1), x2(2), 'b*');
scatter(linspace(-1,1,1000),zeros(1,1000),5,'k');
scatter(linspace(-1,0,1000),2+2*linspace(-1,0,1000),5,'k');
scatter(linspace(0,1,1000),2-2*linspace(0,1,1000),5,'k');
scatter(linspace(-1,x2(1),1000), ...
    x2(2)/(x2(1)+1)*(linspace(-1,x2(1),1000)+1),2,'m*');
scatter(linspace(x2(1),1,1000), ...
    x2(2)/(x2(1)-1)*(linspace(x2(1),1,1000)-1),2,'m*');
scatter(linspace(-1,x1(1),1000), ...
    x1(2)/(x1(1)+1)*(linspace(-1,x1(1),1000)+1),2,'m*');
scatter(linspace(0,x1(1),1000), ...
    (x1(2)-2)/x1(1)*(linspace(0,x1(1),1000))+2,2,'m*');
%% data generation
d11 = norm(x1-A(:,1));
d12 = norm(x1-A(:,2));
d22 = norm(x2-A(:,2));
d23 = norm(x2-A(:,3));
d12h = norm(x1-x2);
%% SOCP
cvx_begin
variables z1(2) z2(2)
minimize(0)
subject to
norm(z1-A(:,1))<=d11
norm(z1-A(:,2))<=d12
norm(z2-A(:,2))<=d22
norm(z2-A(:,3))<=d23
norm(z1-z2)<=d12h
cvx_end
fprintf('x1 error = %3.4e\n', norm(z1-x1));
fprintf('x2 error = %3.4e\n', norm(z2-x2));
scatter(z1(1), z1(2), 'ro', 'filled');
scatter(z2(1), z2(2), 'bo', 'filled');
title('red *: x1; blue *: x2; red o: x1-num; blue o: x2-num');
hold off