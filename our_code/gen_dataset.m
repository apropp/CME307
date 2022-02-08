% Data set generation for Question 1

%% Each column is anchor point
A = [1 -1 0;
    0 0 2];

%%% 1) random initialization choice (remove comment to enable)
x1 = randn(2, 1);
x2 = randn(2, 1);
x3 = randn(2, 1);
x4 = randn(2, 1);
x5 = randn(2, 1);
x6 = randn(2, 1);
x7 = randn(2, 1);
x8 = randn(2, 1);
x9 = randn(2, 1);
x10 = randn(2, 1);

PP = [A x1 x2 x3 x4 x5 x6 x7 x8 x9 x10];
m = 3;
n = 13;
Radius = 2;
nf = 0;
degree = 3;