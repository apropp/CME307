function [d] = distances(PP,m,n,Radius,nf,degree)
%distances Compute distances between points within range.

close all;
    PP=[PP(:,m+1:n) PP(:,1:m)]; % switches around the data so unknowns come first
    dd = n-m; % number of unknowns

    b=zeros(3,1);
    a=sparse([zeros(1,dd) 1 0]');
    A=vec(a*a');
    b(1)=1;
    a=sparse([zeros(1,dd) 0 1]');
    A=[A vec(a*a')];
    b(2)=1;
    a=sparse([zeros(1,dd) 1 1]');
    A=[A vec(a*a')];
    b(3)=2;
    
    % Compute pair-wise distances within R limit between known to unknown 
    % sensor-point and unknown to unknown within the radio range and set up
    % the constraint matrix.
    %
    dij=sparse(zeros(n,n));
    mtotal=3;
    for i=1:dd
        flag = 0 ;
        for j=(i+1):n
            rr= norm(PP(:,i)-PP(:,j));
            if rr < Radius && flag < degree
               flag =flag+1;
               mtotal=mtotal+1;
               dij(i,j)=rr*sqrt(max(0,(1+randn(1)*nf)));%add noise
               if j <= dd
                  % jth is an unknown point (sensor)
                  a=sparse([zeros(1,i-1) 1 zeros(1,j-i-1) -1 zeros(1,dd-j+2)]');
                  A(:,mtotal)=vec(a*a');    
               else
                  % jth is a known point (anchor)
                  a=sparse([zeros(1,i-1) 1 zeros(1,dd-i) -PP(:,j)']');
                  A(:,mtotal)=vec(a*a');
               end;
               b(mtotal) = (dij(i,j))^2; 
            end;
        end;
    end;
end