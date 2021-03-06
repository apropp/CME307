function [Y_opt,X_opt] = SDPsnlsedumi_withoutSteepestDescent(PP,m,n,Radius,nf,degree)
%SDPsnlsedumi_withoutSteepestDescent SDP relaxation using Sedumi package
%   Solves sensor network localization using SDP relaxation. Steepest
%   descent is not applied to the result.
close all;
    PP=[PP(:,m+1:n) PP(:,1:m)]; % switches around the data so unknowns come first
    dd = n-m; % number of unknowns
    Anchor = PP(:,dd+1:n); % anchor locations

    e = [ones(dd,1); sum(PP(:,dd+1:n),2)]/sqrt(n);
    C=1.4*nf*vec(e*e'-speye(dd+2,dd+2));

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
    %
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

    % Setup to call SEDUMI
    mequal=mtotal
    K.s=[dd+2];
    K.s=[K.s ones(1,mtotal-3)];
    pars.eps=1.e-3;
    tic
    A=[A; sparse([zeros(mtotal-3, 3) -eye(mtotal-3)])];
    C=[C;ones(mtotal-3,1)];
    pars.stepdif=0;
    [x,y,info] = sedumi(A,b,C,K,pars);
    Y_opt=mat(x(1:(dd+2)^2));
	X_opt=Y_opt(dd+1:dd+2,1:dd);
    Y_opt=Y_opt(1:dd,1:dd);
    tvar=0;
    sdpt=toc
    % Plot the position estimations produced by the SDP solver
    figure(1);
    hold on ;
    plot(PP(1,dd+1:n),PP(2,dd+1:n),'d');       
    plot(X_opt(1,:),X_opt(2,:),'*r');
    plot(PP(1,1:dd),PP(2,1:dd),'og');
    for i=1:dd
        plot([X_opt(1,i)  PP(1,i)] , [X_opt(2,i)  PP(2,i)]);
    end;
    hold off
    % Compute the variance/trace and true error
    for j=1:dd
        tvar(j)=max(0,Y_opt(j,j)-(X_opt(:,j)'*X_opt(:,j)));
        error(j)=norm(X_opt(:,j)-PP(:,j));
    end;
end