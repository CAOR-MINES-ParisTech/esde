function [zhat, H] = measurement_model_iekf(xe,idx)
fpos = 3+idx*2-1;
zhat = rot(xe(3))'*(xe(fpos:fpos+1)-xe(1:2));
H = zeros(2,size(xe,1));
H(:,1:2) = -rot(xe(3))';
H(:,fpos:fpos+1) =  rot(xe(3))';
end
