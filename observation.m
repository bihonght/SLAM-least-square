function [z] = observation(i, j)
%DISTANCE Summary of this function goes here
%   Detailed explanation goes here

global robot_poses;
global landmarks;

f = landmarks(j,:)';
r = robot_poses(i,:)';
z = [ f(1) - r(1);
      f(2) - r(2)];
    
z = rotation(r)'*z;

end

