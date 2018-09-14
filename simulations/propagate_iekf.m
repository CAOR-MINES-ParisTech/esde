function [xe, Pe, PHI,G] = propagate_iekf(xe,Pe,dt,v_m,omega_m,sigma_v,sigma_w)
J = [0 -1; 1 0];
Q = [sigma_v^2 0 0;0 0 0;0 0 sigma_w^2];

% % propagate state
xe(1:3,1) = [ xe(1) + v_m*dt*cos(xe(3,1));
    xe(2) + v_m*dt*sin(xe(3,1));
    pi_to_pi(xe(3) + omega_m*dt) ];

PHI = eye(size(Pe));
G = eye(length(xe),3);

G(1:2,1:2) = rot(xe(3));
x_temp = -J*vec2mat(xe([1:2 4:end]),2)';
G([1:2 4:end],3) = x_temp(:);

G = G*dt;
Qprime = G*Q*G';
% propagate covariance
Pe = Pe + Qprime;
Pe = 0.5*(Pe+Pe');
% %
PHI = blkdiag(PHI,eye(size(xe,1)-3));
G = 0;
