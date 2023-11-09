function [estimatedZ, estimatedX] = reshape(x)
%RESHAPE_X Summary of this function goes here
%   Detailed explanation goes here
    global num_landmarks
    global num_poses

    flattenZ = x(1:2*num_landmarks); 
    flattenX = x((2*num_landmarks+1):end);
    
    for k=1:num_landmarks
        estimatedZ(k,:) = flattenZ(2*k-1:2*k)';
    end

    for k=2:num_poses
        estimatedX(k,:) = flattenX(3*k-5:3*k-3)';
    end
end

