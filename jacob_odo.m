function [J_odom] = jacob_odo(idx,estimated_X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global size_state_vector
global num_landmarks
global num_poses

% end_X = num_landmarks*2 + 3*idx;
% start_X = end_X - 2;

if (idx == 1) 
    % J_odom = zeros(3,num_poses*3);
    % J_odom = zeros(3,6);
    J_odom(:, 1:3) = [1 0 0; 0 1 0; 0 0 1];
else 
    r          = estimated_X(idx,:)';
    r_previous = estimated_X(idx-1,:)';
    
    dx = r(1) - r_previous(1);
    dy = r(2) - r_previous(2);
    
    % J_odom = ones(3,num_poses*3);
     
    % start_idx = 3*idx - 5; 
    % end_idx = 3*idx; 
    
    J_odom(1,:) = [-cos(r_previous(3)), -sin(r_previous(3)), -dx*sin(r_previous(3))+dy*cos(r_previous(3)), cos(r_previous(3)), sin(r_previous(3)), 0];
    
    J_odom(2,:) = [sin(r_previous(3)), -cos(r_previous(3)), -dx*cos(r_previous(3))-dy*sin(r_previous(3)), -sin(r_previous(3)), cos(r_previous(3)), 0];
    
    J_odom(3,:) = [0, 0, -1, 0, 0, 1];

    J_odom = J_odom(:,1:end);
end

%J_odom = [ones(3,2*num_landmarks), J_odom];

end

