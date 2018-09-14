function [error] = error_angle(angles1, angles2)
error = zeros(length(angles1), 1);
for i = 1:length(angles1)
    R = rot(angles1(i)) * rot(angles2(i))';
    error(i) = atan2(R(2, 1), R(1, 1));
end
end

