function [rotate] = rotation(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

rotate = [cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))];
end

