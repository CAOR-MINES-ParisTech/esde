function [zhat, H] = measurement_model_std(xe,idx)
fpos = 3+idx*2-1;
J = [0 -1; 1 0];
C = rot(xe(3));
k_xL = C'*(xe(fpos:fpos+1)-xe(1:2));
zhat = k_xL;
H = zeros(2,size(xe,1));
H(:,1:3) = - C'*[eye(2)  J*(xe(fpos:fpos+1,1)-xe(1:2,1))];
H(:,fpos:fpos+1)= C';
end

