function [T_inv] = my_inv(T)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
T_inv = eye(4);
T_inv(1:3, 1:3) = T(1:3, 1:3)';
T_inv(1:3, 4) = -T(1:3, 1:3)'*T(1:3, 4);

end

