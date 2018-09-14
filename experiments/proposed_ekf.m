function [estimates] = proposed_ekf(inputs, measurements, ground_truth, timesteps, dt, n_robot, n_landmarks)
disp('starting proposed EKF')
% prealocate variables
estimates.robots = cell(n_robot, 1);
estimates.P = cell(timesteps, 1);
estimates.landmarks = cell(timesteps, 1);
t_odometry = ground_truth.robots{1}(:,1); % time

% init state and covariance
state.robots = cell(n_robot, 1);
state.n_state = 3*n_robot + 3*n_landmarks;
state.P = zeros(state.n_state);
state.Barcodes = ground_truth.Barcodes;
state.threshold = ground_truth.threshold;
state.landmarks = zeros(n_landmarks, 4);
J = [0 -1; 1 0];
for i_robot = 1:n_robot
    state.robots{i_robot} = ground_truth.robots{i_robot}(1, 2:4)';
    idx_robot = 3*(i_robot-1) + (1:3);
    state.P(idx_robot,idx_robot) = diag([0 0 0].^2);
    A = eye(3);
    A(1:2, 3) = -J*state.robots{i_robot}(1:2);
    state.P(idx_robot,idx_robot) = A * state.P(idx_robot,idx_robot) * A';
    estimates.robots{i_robot} = zeros(timesteps, 4);
    estimates.robots{i_robot}(1, :) = [t_odometry(1) state.robots{i_robot}'];
end
estimates.P{1} = state.P;
estimates.landmarks{1} = state.landmarks;

% filtering
for i_step = 1:timesteps-1
    state = propagation(state, inputs, ground_truth.Q, dt, n_robot, n_landmarks, i_step);
    t = t_odometry(i_step + 1);
    state = update(state, measurements, ground_truth.R, t, dt, n_robot, n_landmarks);
    for i_robot = 1:n_robot
        estimates.robots{i_robot}(i_step + 1, :) = [t state.robots{i_robot}'];
    end
    estimates.P{i_step} = state.P;
    estimates.landmarks{i_step} = state.landmarks;
end
end

%---------------------------------------------------------------------------------------------------
function [state] = propagation(state, inputs, Q, dt, n_robot, n_landmarks, i_step)
Q_robots = [];
n_state = state.n_state;
F = zeros(n_state);
G = zeros(n_state, 3*n_robot);
for i_robot = 1:n_robot % propagate each robot
   
    idx_robot = 3*(i_robot-1) + (1:3);
    input = inputs{i_robot}(i_step, 2:3);
    Q_i = zeros(3);
    Q_i(1, 1) = 0.1*abs(input(1));
    Q_i(2, 2) = 0.01*abs(input(1));
    Q_i(3, 3) = 0.1*abs(input(2));
    Q_robots = blkdiag(Q_robots, Q_i);
    [F(idx_robot, idx_robot), G(idx_robot, idx_robot)] = jacobian_robot_propagate(state.robots{i_robot}, ...
        input, dt);
    state.robots{i_robot} = propagate_robot(state.robots{i_robot}, input, dt);
end

Phi = eye(n_state) + F*dt + 1/2*(F*dt)^2 + 1/6*(F*dt)^3;
Q_dt = Phi*G*Q_robots*G'*Phi'*dt;
state.P = Phi*state.P*Phi' + Q_dt; % propagate covariance
end

%---------------------------------------------------------------------------------------------------
function [F, G] = jacobian_robot_propagate(state_robot, input, dt)
J = [0 -1; 1 0];
Rot = rot(state_robot(3));
F = zeros(3);

G = [Rot -J*state_robot(1:2);
    zeros(1, 2) 1];
end

%---------------------------------------------------------------------------------------------------
function [state] = update(state, measurements, R, t, dt, n_robot, n_landmarks)
t_min = t - dt;
t_max = t;
y = []; %the form of each row is [subject, measured subject, r, phi]
for i_robot = 1 :n_robot
    % time selection
    t_measurements = measurements{i_robot}(:, 1);
    idx_measurements = t_measurements > t_min & t_measurements <= t_max;
    y_i = measurements{i_robot}(idx_measurements, :);
    y_i(:, 1) = i_robot; % subject == robot{i_robot}
    y_i(:, 2) = barcodes2subjects(y_i(:, 2), state.Barcodes);
    y = [y; y_i];
end

n_measurements = size(y, 1);
if n_measurements > 0
    H = zeros(2*n_measurements, state.n_state);
    y_hat = zeros(2*n_measurements, 1);
    for i_mes = 1 :n_measurements
        if y(i_mes, 2) > n_robot && y(i_mes, 2) ~= state.landmarks(y(i_mes, 2)-n_robot, 1) % if first time landmark is seen
            i_robot = y(i_mes, 1);
            state_robot = state.robots{i_robot};
            [pos_landmark, state.P] = init_landmark(y(i_mes, 3:4), state_robot, state.P, R, i_robot, ...
                y(i_mes, 2)-n_robot, n_robot);
            state.landmarks(y(i_mes, 2)-n_robot, :) = [y(i_mes, 2) pos_landmark'];
        end
        
        idx_mes = 2*(i_mes-1) + (1:2);
        H(idx_mes, :) = jacobian_robot_update(state, y(i_mes, 1), ...
            y(i_mes, 2), n_robot);
        y_hat(idx_mes) = measurement_model(state, y(i_mes, 1), ...
            y(i_mes, 2), n_robot);
    end
    S = H*state.P*H' + kron(eye(n_measurements), R);
    K = state.P*H'/S;
    y_mesasurement = y(:, 3:4)';
    residual = y_mesasurement(:)-y_hat;
    
    % chi_2 test
    for i_mes = n_measurements:-1:1
        idx_mes = 2*(i_mes-1) + (1:2);
        if residual(idx_mes)'*(S(idx_mes, idx_mes)\residual(idx_mes))/2 > state.threshold
            K(:, idx_mes) = [];
            residual(idx_mes) = [];
            H(idx_mes, :) = [];
        end
    end
    if size(K, 2) > 0
        state = update_state(K*residual, state, n_robot, n_landmarks);
        state.P = (eye(size(state.P)) - K*H)*state.P;
    end
end
end

%---------------------------------------------------------------------------------------------------
function [H] = jacobian_robot_update(state, subject, measured_subject, n_robots)
J = [0 -1; 1 0];
x_subject = state.robots{subject}(1:2);
Rot = rot(state.robots{subject}(3));
if measured_subject <= n_robots % robot
    x_measured_subject = state.robots{measured_subject}(1:2);
else % landmarks
    x_measured_subject = state.landmarks(state.landmarks(:, 1) == measured_subject, 2:3)';
end
x2x_measumed = Rot'*(x_measured_subject-x_subject);

H = zeros(2, state.n_state);
DH = [1/norm(x2x_measumed)*x2x_measumed'; 1/norm(x2x_measumed)^2*x2x_measumed'*J'];

H(:, 3*(subject-1) + (1:3)) = DH*[-Rot'  -Rot'*J*x_measured_subject];
H(:, 3*(measured_subject-1) + (1:3)) = DH*[Rot'  Rot'*J*x_measured_subject];

end

%---------------------------------------------------------------------------------------------------
function [state] = update_state(innov, state, n_robots, n_landmarks)
for i_robot = 1:n_robots
    idx_robot = 3*(i_robot-1) + (1:3);
    state.robots{i_robot} = phi(state.robots{i_robot}, innov(idx_robot));
end

for i_landmark = 1:n_landmarks
    idx_landmark = 3*n_robots + 3*(i_landmark-1) + (1:3);
    state.landmarks(i_landmark, 2:4) = phi(state.landmarks(i_landmark, 2:4)', innov(idx_landmark))';
end
end

%---------------------------------------------------------------------------------------------------
function [pos_landmark, P] = init_landmark(y, state_robot, P, R, i_robot, i_landmark, n_robots)

% first landmark estimate
r = y(1);
phi = y(2);
pos_robot2landmark = r*[cos(phi); sin(phi)];

pos_landmark = [state_robot(1:2) + r*[cos(phi+state_robot(3)); sin(phi+state_robot(3))];
    state_robot(3)];

% compute Jacobians
J = [0 -1; 1 0];
Rot = rot(state_robot(3));
DH = [ 1/norm(pos_robot2landmark)*pos_robot2landmark'; 1/norm(pos_robot2landmark)^2*pos_robot2landmark'*J' ];

JJ = eye(size(P));
JJ(3*n_robots + 3*(i_landmark-1) + 3, 3*(i_robot-1) + 3) = 1;
JJ(3*n_robots + 3*(i_landmark-1) + 3, 3*n_robots + 3*(i_landmark-1) + 3) = 0;
P = JJ*P*JJ';

idx_robot = 3*(i_robot-1) + (1:2);
H_robot = -DH*Rot';
H_landmark = DH*Rot';

% augment covariance;
idx_landmark = 3*n_robots + 3*(i_landmark-1) + (1:2);
P_robot = P(idx_robot, idx_robot);
H_il = H_landmark^-1;

P(idx_landmark, idx_landmark) = H_il*H_robot*P_robot*H_robot'*H_il' + ...
    H_il*R*H_il'; % landmark cov
P(idx_landmark, idx_robot) = -H_il*H_robot*P_robot; % landmark-robot correlation
P(idx_robot, idx_landmark) = P(idx_landmark, idx_robot)';

% remainining cross correlation
idx_rnm = 1:size(P, 1);
idx_rnm([idx_robot idx_landmark]) = [];
P(idx_landmark, idx_rnm) = -H_il*H_robot*P(idx_robot, idx_rnm);
P(idx_rnm, idx_landmark) = P(idx_landmark, idx_rnm)';
end