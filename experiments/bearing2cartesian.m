function [pos_observed] = bearing2cartesian(y, state_robot)
r = y(1);
phi = y(2);
pos_observed = state_robot(1:2) + r*[cos(phi+state_robot(3)); sin(phi+state_robot(3))];
end

