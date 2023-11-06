function [estimatedX, estimatedZ] = slam(i,j, X, Z)
%SLAM Summary of this function goes here
%   Detailed explanation goes here

odo = odometry(i);                      % odometry from pose r(i-1) to r(i) 
z1 = observation(i-1, j);               % observation from pose r(i-1) to f(j)
z2 = observation(i, j);                 % observation from pose r(i) to f(j)

                            % number of epoch 10

    K = [odo; z1; z2]; % observation

    % for i
    H = [H_odo(i,X); H_Z(i-1,j,Z,X); H_Z(i,j,Z,X)];
    
    J = jacob(i, j, X, Z);
    
    x = [Z(j, :) X(i,:)]; 
    x = (inv(J'*J)*J'*(K - H + J*x'))'
    Z(j,:) = x(:,1:2);
    X(i,:) = x(:,3:5);
    
    estimatedX = X; 
    estimatedZ = Z;


end

