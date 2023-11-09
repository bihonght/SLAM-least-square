function [output] = H_odo(idx,estimated_X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global num_landmarks

r = estimated_X(idx,:)';

if (idx == 1)
   output = r;
else 
    r_previous = estimated_X(idx-1,:)';
    distX = r(1) - r_previous(1);
    distY = r(2) - r_previous(2);
    theta = wrapToPi(r(3) - r_previous(3));
    output = [rotation(r_previous)'*[distX; distY]; theta]; 
end

end

