function [estimatedX, estimatedZ] = slam(X, Z)
%SLAM Summary of this function goes here
%   Detailed explanation goes here
    global num_poses
    global num_landmarks

    K = [];     % observation_data
    H = [];     % function_data
    J = [];     % jacobian_function
    for i=2:num_poses   
        K = [K; odometry(i)];               % odometry calculation
        H = [H; H_odo(i,X)];
        J = [J; jacob_odo(i,X)];
        for j=1:num_landmarks
            K = [K; observation(i, j)];     % landmarks calculation
            H = [H; H_Z(i,j,X,Z)];
            J = [J; jacob_z(i,j,X,Z)];
        end
    end
    
    % size(J)
    x = [reshape(Z', [], 1); reshape(X(2:end,:)', [], 1)]   % flatten vector X,Z 
    % solution weighted non-linear least squares
    x = (inv(J'*J)*J'*(K - H + J*x))

    [estimatedZ, estimatedX] = reshape_XZ(x);
end

