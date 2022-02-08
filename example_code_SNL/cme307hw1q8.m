%% MS&E 311/CME 307 Homework 1 Problem 8

%% Each column is anchor point
A = [1 -1 0;
    0 0 2];

%% Generate sensor in convex hull of 3 anchors
%% SOCP relaxation
alpha = rand(3,1);
alpha = alpha/norm(alpha,1);
s_true = A*alpha;
d = norms(A - s_true*ones(1,3));

cvx_begin quiet
    variable s(2)
    minimize( 0 )
    subject to
    norms(A - s*ones(1,3)) <= d;
cvx_end

fprintf('SOCP - Inside of Convex Hull\n');
fprintf('True sensor location : (%f, %f)\n', s_true(1), s_true(2));
fprintf('Recovered sensor location: (%f, %f)\n', s(1), s(2));
fprintf('Difference : %f\n\n', norm(s_true - s));

%% SDP relaxation
cvx_begin sdp quiet
    variable X(3,3) semidefinite
    minimize( 0 )
    subject to
        X(1:2,1:2) == eye(2)
        for i=1:3
            [A(:,i);-1]'*X*[A(:,i);-1] == d(i)^2
        end
cvx_end

s = X(1:2,3);

fprintf('SDP - Inside of Convex Hull\n');
fprintf('True sensor location : (%f, %f)\n', s_true(1), s_true(2));
fprintf('Recovered sensor location: (%f, %f)\n', s(1), s(2));
fprintf('Difference : %f\n\n', norm(s_true - s));

%% Generate sensor outside of convex hull of 3 anchors
alpha = 10*rand(3,1);
s_true = A*alpha;
d = norms(A - s_true*ones(1,3));

cvx_begin quiet
    variable s(2)
    minimize( 0 )
    subject to
        norms(A - s*ones(1,3)) <= d;
cvx_end

fprintf('SOCP - Outside of Convex Hull\n');
fprintf('True sensor location : (%f, %f)\n', s_true(1), s_true(2));
fprintf('Recovered sensor location: (%f, %f)\n', s(1), s(2));
fprintf('Difference : %f\n\n', norm(s_true - s));

%% SDP relaxation
cvx_begin sdp quiet
    variable X(3,3) semidefinite
    minimize( 0 )
    subject to
        X(1:2,1:2) == eye(2)
        for i=1:3
            [A(:,i);-1]'*X*[A(:,i);-1] == d(i)^ 2
        end
cvx_end

s = X(1:2,3);

fprintf('SDP - Outside of Convex Hull\n');
fprintf('True sensor location : (%f, %f)\n', s_true(1), s_true(2));
fprintf('Recovered sensor location: (%f, %f)\n', s(1), s(2));
fprintf('Difference : %f\n\n', norm(s_true - s));