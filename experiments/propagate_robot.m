function [state_robot] = propagate_robot(state_robot, input, dt)
Rot = rot(state_robot(3));
v = [input(1); 0];
omega = input(2);
state_robot = state_robot + [Rot*v;
                             omega]*dt;
state_robot(3) = pi2pi(state_robot(3));                    
end

