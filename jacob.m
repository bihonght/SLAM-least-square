function [J] = jacob(id_r,id_f,estimated_X, estimated_f)
%JACOB Summary of this function goes here
%   Detailed explanation goes here

r          = estimated_X(id_r,:)';
r_previous = estimated_X(id_r-1,:)';
f = estimated_f(id_f,:)';

J = zeros(7,5);
% odometry jacobian from pose r-1 to pose r
J(1,:) = [0, 0, cos(r_previous(3)), sin(r_previous(3)), 0];
    
J(2,:) = [0, 0, -sin(r_previous(3)), cos(r_previous(3)), 0];
    
J(3,:) = [0, 0, 0, 0, 1];

% observation jacobian from the pose r-1 to the feature f

J(4,:) = [cos(r_previous(3)), sin(r_previous(3)), 0, 0, 0];

J(5,:) = [-sin(r_previous(3)), cos(r_previous(3)), 0, 0, 0];

% observation jacobian from the pose r to the feature f
dxz = -r(1) + f(1);
dyz = -r(2) + f(2);

J(6,:) = [cos(r(3)), sin(r(3)), -cos(r(3)), -sin(r(3)), -dxz*sin(r(3))+dyz*cos(r(3))];

J(7,:) = [-sin(r(3)), cos(r(3)), sin(r(3)), -cos(r(3)), -dxz*cos(r(3))-dyz*sin(r(3))];


end

