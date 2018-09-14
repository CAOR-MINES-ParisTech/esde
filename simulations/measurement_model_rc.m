function [zhat, H] = measurement_model_rc(xe,v_m,omega_m,idx)
% assume dt = 1;
dt = 1;
omega_m = omega_m*dt;
v_m = v_m*dt;
J = [0 -1; 1 0];


fpos = 3+idx*2-1;


p_propagated = xe(fpos:fpos+1)-[v_m;0];
zhat = rot(omega_m)'*p_propagated;


H = zeros(2,size(xe,1)+3);


H(:,fpos:fpos+1)= rot(omega_m)';
H(:,end-2) = J*rot(omega_m)'*([v_m;0]-xe(fpos:fpos+1));
H(:,end-1:end) = -rot(omega_m)';
end

