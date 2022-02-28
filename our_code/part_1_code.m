clear all; close all;

%% Initialization
rng(2022); % set seed
d = 2; 
n_anchors = 5; % known locations
n_sensors = 10; % number of sensors
% n unknown locations = n_sensors - n_anchors
radius = 0.3; % range within which distances are known
A = rand(2,n_anchors); % n_anchors random points in coordinate plane
X = rand(2,n_sensors); % n_sensors random points in the coordinate plane
noise_factor = 0; % noise in distance estimation
degree = 7; %max number of edge distances used per node

% D_{ij} is the distance between point i and point j, where i,j in [1, 5] 
% is an anchor and i,j in [6, 15] is a sensor. 
D = squareform(pdist([A,X]'));

%% Q1
% SDP relaxation
%[Y_opt_SDP,X_opt_SDP]=SDPsnlsedumi_withoutSteepestDescent(A,n_anchors,n_sensors,radius,noise_factor,degree)


% SOCP relaxation
cvx_begin quiet
    variable Z(d, n_sensors)
    minimize( 0 )
        subject to
        for i=1:n_sensors
            norms(A - Z(:, i)*ones(1, n_anchors)) <= ...
                D(n_anchors+i, 1:n_anchors).^2;
            norms(Z - Z(:, i)*ones(1, n_sensors)) <= ...
                D(n_anchors+i, n_anchors+1:end).^2;
        end
cvx_end

m = d + n_sensors; 
Id = eye(m - d); 
% SDP relaxation
cvx_begin quiet
    variable Z(m, m) semidefinite
    minimize( 0 )
    subject to
        Z(1:d, 1:d) == eye(d); 
        for i=1:n_sensors
            for j = i+1:n_sensors
                (Id(:, i) - Id(:, j))'*Z(d+1:end, d+1:end)*...
                    (Id(:, i) - Id(:, j)) == D(n_anchors+i, n_anchors+j)^2;
            end
        end
        for k = 1:n_anchors
            for j = 1:n_sensors
                [A(:, k); -Id(:, j)]'*Z*[A(:, k); -Id(:, j)] ...
                    == D(k, n_anchors+j)^2;
            end
        end
%         Z(d+1:end, d+1:end) == Z(1:d, d+1:end)'*Z(1:d, d+1:end); 
cvx_end

Z = Z(1:d, d+1:end);


figure(1);
hold on ;
% anchors
plot(A(1, :), A(2, :), 'd'); 
% sensors
plot(X(1, :), X(2, :), 'og');
% 
plot(Z(1, :), Z(2, :), '*r'); 

% plot(PP(1,dd+1:n),PP(2,dd+1:n),'d');       
% plot(X_opt(1,:),X_opt(2,:),'*r');
% plot(PP(1,1:dd),PP(2,1:dd),'og');
for i=1:n_sensors
    plot([Z(1,i)  X(1,i)] , [Z(2,i)  X(2,i)]);
end
hold off

