% Lec 1 P6.1 LP
cvx_begin
variable x(3)
minimize 2*x(1)+x(2)+x(3)
subject to
    sum(x)==1;
    x>=0;
cvx_end

% SOCP
cvx_begin
variable x(3)
minimize 2*x(1)+x(2)+x(3)
subject to
    sum(x)==1;
    %sqrt(x(2)^2+x(3)^2)<=x(1); % wrong! not satisfies DCP
    norm(x(2:3))<=x(1);
cvx_end

% SDP
cvx_begin
variable x(3)
variable Z(2,2) semidefinite
minimize 2*x(1)+x(2)+x(3)
subject to
    sum(x) == 1;
    Z(1,1) == x(1);
    Z(1,2) == x(2);
    Z(2,2) == x(3);
cvx_end

% P13-15 Supporting Vector Machine: Ellipsoidal Separation:
% roughly generate the data
n = 100;
data = 10*rand(2,n);
index_1 = find(norms(data-[5;5]*ones(1,n))<=2);
index_2 = find(norms(data-[5;5]*ones(1,n))>2);

plot(data(1,index_1),data(2,index_1),'o')
hold on
plot(data(1,index_2),data(2,index_2),'o')
hold on
plot(5+2*sin((1:n)/n*2*pi),5+2*cos((1:n)/n*2*pi))
hold on

cvx_begin quiet
variables x(2) x0
variable X(2,2) semidefinite
minimize (trace(X)+sum(([x;x0])).^2)
subject to
    diag(data(:,index_1)'*X*data(:,index_1)) + data(:,index_1)'*x +x0  <= -1
    diag(data(:,index_2)'*X*data(:,index_2)) + data(:,index_2)'*x +x0  >= 1
cvx_end


