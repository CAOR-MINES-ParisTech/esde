function x_plus_dx = phi(x, dx)

X = dx(1:2);
theta = dx(3);

if theta>1E-8
    B = [sin(theta)/theta  -(1-cos(theta))/theta; 
        (1-cos(theta))/theta sin(theta)/theta];
else
    sin_theta_sur_theta = 1-theta^2/6 + theta^4/120;
    un_moins_cos_theta_sur_theta = theta/2 - theta^3/24 + theta^5/720;
    B = [sin_theta_sur_theta -un_moins_cos_theta_sur_theta;
        un_moins_cos_theta_sur_theta sin_theta_sur_theta];
end


exp_dx = [B*X; theta];
x_plus_dx = [rot(dx(3))*x(1:2) + exp_dx(1:2);
    x(3) + exp_dx(3)];
end