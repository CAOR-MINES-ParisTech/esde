function [zhat, H] = measurement_model_ocekf(xe,idx,V_1,xL_1,dpR,lambda_i)
fpos = 3+idx*2-1;
J = [0 -1; 1 0];
C = rot(xe(3));
k_xL = C'*(xe(fpos:fpos+1)-xe(1:2));
zhat = k_xL;
% % Jacobian
H = zeros(2,size(xe,1));
pL_star = xe(fpos:fpos+1,1)-lambda_i/2;
H(:,1:3) = - C'*[eye(2)  J*(pL_star-xe(1:2,1))];
H(:,fpos:fpos+1) = C';
end

