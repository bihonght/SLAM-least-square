function [output] = H_Z(id_r, id_f,estimated_X, estimated_f)
%H_Z Summary of this function goes here
%   Detailed explanation goes here
f = estimated_f(id_f,:)';
r = estimated_X(id_r,:)';

if (id_r == 1)
   output = f;
else 
    distX = f(1) - r(1);
    distY = f(2) - r(2);
    output = rotation(r)'*[distX; distY];
end


end

