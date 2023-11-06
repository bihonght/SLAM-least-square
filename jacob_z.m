function [J_z] = jacob_z(id_r,id_f,estimated_X, estimated_f)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

r = estimated_X(id_r,:)';
f = estimated_f(id_f,:)';

dx = -r(1) + f(1);
dy = -r(2) + f(2);

% if (idr == 1) 
%     % J_odom = zeros(3,num_poses*3);
%     % J_odom = zeros(3,6);
% 
%     J_z = [1, 0, -1, 0, dy; 0, 1, 0, -1, -dx]
% 
% 
% else 
%     % r_previous = estimated_X(idx-1,:)';
% 
% 
% 
%     % J_odom = ones(3,num_poses*3);
% 
%     % start_idx = 3*idx - 5; 
%     % end_idx = 3*idx; 
    
    J_z(1,:) = [cos(r(3)), sin(r(3)), -cos(r(3)), -sin(r(3)), -dx*sin(r(3))+dy*cos(r(3))];

    J_z(2,:) = [-sin(r(3)), cos(r(3)), sin(r(3)), -cos(r(3)), -dx*cos(r(3))-dy*sin(r(3))];

    
    % J_z = J_z(:,1:2)
% end

%J_odom = [ones(3,2*num_landmarks), J_odom];

end

