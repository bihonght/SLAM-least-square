function [Odo] = odometry(i)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global robot_poses;

r = robot_poses(i,:)';
r_previous = robot_poses(i-1,:)';

distX = r(1) - r_previous(1);
distY = r(2) - r_previous(2);
theta = wrapToPi(r(3) - r_previous(3));

% Odo = [distX; distY; theta];

Odo = [rotation(r_previous)'*[distX; distY]; theta]; 

end

