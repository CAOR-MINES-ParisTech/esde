clear all
close all

%% select datasets, modes and filters
n_dataset = 9;
n_robot = 5;
filters = ["standard EKF","proposed EKF"];

%% prealocate variables
estimates_main  = cell(n_dataset, length(filters));
ground_truth_main = cell(n_dataset);

%% main part
for i_dataset = 1:n_dataset
    dir_dataset = char("MRCLAM" + i_dataset);
    addpath(dir_dataset)
    loadMRCLAMdataSet
    sampleMRCLAMdataSet
    data2input
    ground_truth_main{i_dataset} = ground_truth;
    estimates_main{i_dataset, 1} = standard_ekf(inputs, measurements, ...
        ground_truth, timesteps, sample_time, n_robot, ...
                    n_landmarks);
    estimates_main{i_dataset, 2} = proposed_ekf(inputs, measurements, ...
        ground_truth, timesteps, sample_time, n_robot, ...
                            n_landmarks);
    rmpath(dir_dataset)
end

%% compute rmse
rmses_main = cell(n_dataset, length(filters));
rmse_main = cell(n_dataset, length(filters));

% compute rmse
for i_dataset = 1:n_dataset
    ground_truth = ground_truth_main{i_dataset};
    for i_filter = 1:length(filters)
        rmses_main{i_dataset, i_filter} = cell(n_robot);
        estimates = estimates_main{i_dataset, i_filter};
        for i_robot = 1:n_robot
            g_t_robot = ground_truth.robots{i_robot}(:,2:4);
            est_robot = estimates.robots{i_robot}(:, 2:4);
            error = g_t_robot-est_robot;
            error(:, 3) = error_angle(g_t_robot(:, 3), est_robot(:, 3));
            rmse = sqrt((error).^2);
            rmses_main{i_dataset, i_filter}{i_robot} = rmse;
        end
    end
end

% average over n_robot
for i_filter = 1:length(filters)
    rmse_f = 0;
    for i_dataset = 1:n_dataset
        rmse_main{i_dataset, i_filter} = zeros(1, 3);
        for i_robot = 1:n_robot
            rmse_main{i_dataset, i_filter} = rmse_main{i_dataset,  i_filter} + ...
                mean(rmses_main{i_dataset, i_filter}{i_robot}.^2, 1)/n_robots;
        end
        rmse_f = rmse_f + rmse_main{i_dataset, i_filter}/n_dataset;
    end
end


%% plot and save
n_dataset = 9;
rmse = zeros(n_dataset, 2);
for i =1:2
    for j = 1:n_dataset
        rmse(j,i) = sqrt(mean(rmse_main{j, i}(1:2)));
    end
end
figure()
bar((1:n_dataset)', rmse, 1)
set(gca, 'XTick', 1:n_dataset)
title('Robot position RMSE (m)','Interpreter','latex')
ylabel('robot position RMSE (m)','Interpreter','latex')
xlabel('experiment number','Interpreter','latex')
legend('standard EKF', 'proposed EKF')

disp(mean(rmse(1:9,1)-rmse(1:9,2)))
disp(mean(rmse(1:8,1)-rmse(1:8,2)))
save('results.mat', 'rmse_main')

