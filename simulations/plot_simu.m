close all
 
% % setup 
nr= 1;
figure
plot(xR_true(1,:,nr), xR_true(2,:,nr), 'r-','Linewidth',1), hold on
plot(xL_true_fixed(1,:), xL_true_fixed(2,:), 'b*','Linewidth',1)
title('Robot trajectory and landmarks','Interpreter','latex')
legend('True Trajectory','True Landmarks','Interpreter','latex')
xlabel('x (m)','Interpreter','latex'), ylabel('$y$ (m)','Interpreter','latex')
axis equal

% % robot % %
start= 5;  incr= 1;

% Robot NEES
figure, hold on
plot([start:incr:nSteps],neesR_avg_std(start:incr:end)','b-','Linewidth',2);
plot([start:incr:nSteps],neesR_avg_ocekf(start:incr:end)','r-','Linewidth',2);
plot([start:incr:nSteps],neesR_avg_rc(start:incr:end)','c-','Linewidth',2);
plot([start:incr:nSteps],neesR_avg_iekf(start:incr:end)','g-','Linewidth',2);
xlabel('time steps','Interpreter','latex');
title('Robot pose NEES','Interpreter','latex');
ylabel('Robot pose NEES','Interpreter','latex')
if is_isam
    plot([start:incr:nSteps],neesR_avg_isam(start:incr:end)','y-','Linewidth',2);
    legend('standard EKF','OC-EKF','robocentric','proposed EKF', 'iSAM','Interpreter','latex')
else
    legend('standard EKF','OC-EKF','robocentric','proposed EKF','Interpreter','latex')
end

start= 1;  incr= 1;
% Robot RMSE
figure
subplot(2,1,1), hold on
plot([start:incr:nSteps],rmsRp_avg_std(start:incr:end)','b-','Linewidth',2);
plot([start:incr:nSteps],rmsRp_avg_ocekf(start:incr:end)','r-','Linewidth',2);
plot([start:incr:nSteps],rmsRp_avg_rc(start:incr:end)','c-','Linewidth',2);
plot([start:incr:nSteps],rmsRp_avg_iekf(start:incr:end)','g-','Linewidth',2);
title('Robot position RMSE (m)','Interpreter','latex')
ylabel('robot position RMSE (m)','Interpreter','latex')
if is_isam
    plot([start:incr:nSteps],rmsRp_avg_isam(start:incr:end)','y-','Linewidth',2);
    legend('standard EKF','OC-EKF','robocentric','proposed EKF', 'iSAM','Interpreter','latex')
else
    legend('standard EKF','OC-EKF','robocentric','proposed EKF','Interpreter','latex')
end

subplot(2,1,2), hold on
plot([start:incr:nSteps],rmsRth_avg_std(start:incr:end)','b-','Linewidth',2);
plot([start:incr:nSteps],rmsRth_avg_ocekf(start:incr:end)','r-','Linewidth',2);
plot([start:incr:nSteps],rmsRth_avg_rc(start:incr:end)','c-','Linewidth',2);
plot([start:incr:nSteps],rmsRth_avg_iekf(start:incr:end)','g-','Linewidth',2);
xlabel('time steps','Interpreter','latex')
title('Robot heading RMSE (rad)','Interpreter','latex')
ylabel('heading RMSE (rad)','Interpreter','latex')
if is_isam
    plot([start:incr:nSteps],rmsRth_avg_isam(start:incr:end)','y-','Linewidth',2);
end


if is_isam
    start= 2;  incr= 1;
    figure, hold on
    plot([start:incr:nSteps],dist2sam_avg_std(start:incr:end)','b-','Linewidth',2);
    plot([start:incr:nSteps],dist2sam_avg_ocekf(start:incr:end)','r-','Linewidth',2);
    plot([start:incr:nSteps],dist2sam_avg_rc(start:incr:end)','c-','Linewidth',2);
    plot([start:incr:nSteps],dist2sam_avg_iekf(start:incr:end)','g-','Linewidth',2);
    xlabel('time steps','Interpreter','latex')
    title('distance to iSAM (m)','Interpreter','latex')
    ylabel('distance to iSAM (m)','Interpreter','latex')
    legend('standard EKF','OC-EKF','robocentric','proposed EKF','Interpreter','latex')
end

save('results.mat')

start = 5; incr=2;
p= [start:incr:nSteps;neesR_avg_std(start:incr:end);
    neesR_avg_ocekf(start:incr:end);
    neesR_avg_rc(start:incr:end);
    neesR_avg_iekf(start:incr:end);
    neesR_avg_isam(start:incr:end)]';
save('fig2.txt','p','-ascii')


start = 5; incr=2;
p= [start:incr:nSteps;rmsRp_avg_std(start:incr:end);
    rmsRp_avg_ocekf(start:incr:end);
    rmsRp_avg_rc(start:incr:end);
    rmsRp_avg_iekf(start:incr:end);
    rmsRp_avg_isam(start:incr:end)]';
save('fig3_position.txt','p','-ascii')

start = 5; incr=2;
p= [start:incr:nSteps;dist2sam_avg_std(start:incr:end);
    dist2sam_avg_ocekf(start:incr:end);
    dist2sam_avg_rc(start:incr:end);
    dist2sam_avg_iekf(start:incr:end)]';
save('fig4.txt','p','-ascii')