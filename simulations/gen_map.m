function xL = gen_map(nL,v,omega,rmin,rmax, nSteps,dt)
% radius
cy = 3.3318;
cx = 0.1251;
r = v/omega;
for i=1:nL
    rho = r + 2*rmin;
    th =   2*pi*i/nL;
    [x,y] = pol2cart(th,rho);
    xL(:,i) = [x+cx;y+cy]; %shift y w/ r since robot starts at (0,0)
end
end

