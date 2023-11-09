function [J_odom] = jacob_odo(idx,estimated_X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global size_state_vector
global num_landmarks
global num_poses

end_id = num_landmarks*2 + 3*idx-3;

J_odom = zeros(3,(num_poses-1)*3 + num_landmarks*2);

 r          = estimated_X(idx,:)';
 r_previous = estimated_X(idx-1,:)';

if (idx == 2) 
    start_id = end_id - 2;
    J_odom(1, start_id:end_id) = [cos(r_previous(3)), sin(r_previous(3)), 0];
    J_odom(2, start_id:end_id) = [-sin(r_previous(3)), cos(r_previous(3)), 0];
    J_odom(3, start_id:end_id) = [0, 0, 1];
else 
    dx = r(1) - r_previous(1);
    dy = r(2) - r_previous(2);
    start_id = end_id - 5;
    J_odom(1, start_id:end_id) = [-cos(r_previous(3)), -sin(r_previous(3)), -dx*sin(r_previous(3))+dy*cos(r_previous(3)), cos(r_previous(3)), sin(r_previous(3)), 0];
    J_odom(2, start_id:end_id) = [sin(r_previous(3)), -cos(r_previous(3)), -dx*cos(r_previous(3))-dy*sin(r_previous(3)), -sin(r_previous(3)), cos(r_previous(3)), 0];
    J_odom(3, start_id:end_id) = [0, 0, -1, 0, 0, 1];
end

%J_odom = [ones(3,2*num_landmarks), J_odom];

end

