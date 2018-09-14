function [xe_rc,P_ext] = propagate_rc(xe_rc,Pe_rc,dt,v_m,omega_m,sigma_v,sigma_w)
Q = [sigma_w^2 0 0;0 sigma_v^2 0;0 0 0];
P_ext = blkdiag(Pe_rc,Q);
end

