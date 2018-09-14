ground_truth.robots = cell(n_robot, 1);
ground_truth.landmarks = Landmark_Groundtruth;

ground_truth.Barcodes = Barcodes;
if ismember(i_dataset, 1) % error in original dataset
    ground_truth.Barcodes([11 17], 2) = Barcodes([17 11], 2);
end

inputs = cell(n_robot, 1);
measurements = cell(n_robot, 1);
for i_robot = 1:n_robot
    ground_truth.robots{i_robot} = eval("Robot" + i_robot + "_Groundtruth");
    inputs{i_robot} = eval("Robot" + i_robot + "_Odometry");
    measurements{i_robot} = eval("Robot" + i_robot + "_Measurement");
    % remove uncorrect measurement
     measurements{i_robot} = clean_measurements(measurements{i_robot},ground_truth.Barcodes);
     ground_truth.robots{i_robot}(1, 2:4) = ground_truth.robots{i_robot}(1, 2:4);
end
% noise_calibration
Q = diag([0.1, 0, 0.1].^2);
R = diag([0.5, 3*pi/180].^2);
ground_truth.threshold = 15;
ground_truth.Q = Q; % unused
ground_truth.R = R;
