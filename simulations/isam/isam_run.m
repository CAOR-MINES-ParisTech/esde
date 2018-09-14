mkdir('isam/_Aux')
rmdir('isam/_Aux', 's')
mkdir('isam/_Aux')
Create_SAM_input
isam_exe_rep = 'isam/bin';
options = '-C';
cmd_line = [isam_exe_rep,'/isam ', options, ' -qW ', 'isam/_Aux/temp_isam ',isam_input_file_name];
system(cmd_line);
extract_isam_results 

theta_SAM = unwrap(res_poses(1,:));
x_SAM = res_poses(2:3,:);
xe_isam = [x_SAM; theta_SAM];
for k = 1:nSteps-1
    err = xR_true(1:3,k+1,kk) - xe_isam(1:3,k);
    err(3) = pi_to_pi(err(3));
    neesR_isam(:,k+1,kk) = err'*inv(Pe_isam(1:3,1:3,k))*err;
    rmsRp_isam(:,k+1,kk) = err(1:2,1)'*err(1:2,1);
    rmsRth_isam(:,k+1,kk) = err(3,1)'*err(3,1);
   
end

x_fit_std = match_map(xRest_std(1:2,2:end,kk),xe_isam(1:2,1:end-1));
x_fit_ocekf = match_map(xRest_ocekf(1:2,2:end,kk),xe_isam(1:2,1:end-1));
x_fit_rc = match_map(xRest_rc(1:2,2:end,kk),xe_isam(1:2,1:end-1));
x_fit_iekf = match_map(xRest_iekf(1:2,2:end,kk),xe_isam(1:2,1:end-1));
dist2sam_std(:, 2:end, kk) = sqrt(mean((x_fit_std-xe_isam(1:2,1:end-1)).^2));
dist2sam_ocekf(:, 2:end, kk) = sqrt(mean((x_fit_ocekf-xe_isam(1:2,1:end-1)).^2));
dist2sam_rc(:, 2:end, kk) = sqrt(mean((x_fit_rc-xe_isam(1:2,1:end-1)).^2));
dist2sam_iekf(:, 2:end, kk) = sqrt(mean((x_fit_iekf-xe_isam(1:2,1:end-1)).^2));