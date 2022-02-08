%======================================================
%  Matlab Demonstration of the LP formulation of the
%  4-City WBC example with a single demand distribution 
%
% Lecture Note #1 of MSandE111X/211X
%
% Construct the transportation matrix for a single demand
% distribution 
A=sparse([ones(1,4) zeros(1,4) zeros(1,4) zeros(1,4);
   zeros(1,4) ones(1,4) zeros(1,4) zeros(1,4);
   zeros(1,4) zeros(1,4) ones(1,4) zeros(1,4);
   zeros(1,4) zeros(1,4) zeros(1,4) ones(1,4);
   eye(4) eye(4) eye(4) eye(4)]);
% Remove the last row since it is redundant
A=A(1:7,:);
% Input the objective coefficientvector for a single demand
c=[0 1 1 2 1 0 2 1 1 2 0 1 2 1 1 0]';
% Single given demand distributions
%d=[3;3;3;0]; %left demand distribution
%d=[0;3;3;3]; %middle demand distribution
d=[3;0;3;3]; %right demand distribution
% Supply distribution
s=[2;2;3;2];
% Construct the RHS vector
b=[s;d(1:3)];
% Call LP solver
[x,y] = HSDLPsolver(A,b,c);
% 