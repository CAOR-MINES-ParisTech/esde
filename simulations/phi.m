function res=phi(dx,x)
N = (length(dx)-1)/2;
theta = dx(1);
X = reshape(dx(2:2*N+1),2,N);

% Matrix B is computed either through its formula given in the paper, or a
% Taylor expansion is theta is too mall (this avoids dividing by zero)
if theta>1E-5
    B = [[sin(theta)/theta , -(1-cos(theta))/theta];[(1-cos(theta))/theta,sin(theta)/theta]];
else
    sin_theta_sur_theta = 1-theta^2/6 + theta^4/120;
    un_moins_cos_theta_sur_theta = theta/2 - theta^3/24 + theta^5/720;
    B = [[sin_theta_sur_theta , -un_moins_cos_theta_sur_theta];[un_moins_cos_theta_sur_theta,sin_theta_sur_theta]];    
end
exp_dx = [theta;reshape(B*X,2*N,1)];

res = zeros(1+2*N,1);
res(1) = x(1) + exp_dx(1);

res_aux = rot(dx(1))*reshape(x(2:end),2,N) + reshape(exp_dx(2:end),2,N);
res(2:end) = res_aux(:);    
end