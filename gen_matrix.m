function [outputArg1] = gen_matrix(m,n)
%GEN_MATRIX Summary of this function goes here
%   Detailed explanation goes here
A = zeros(m, n);

% Generate a random matrix with values between 0 and 1
while true
    A = rand(m, n);
    
    % Check if the matrix is full rank (invertible)
    if rank(A) == min(m, n)
        break;
    end
end
end

