function y_hat = measurement_model(state, subject, measured_subject, n_robots)
x_subject = state.robots{subject}(1:2);
Rot = rot(state.robots{subject}(3));

if measured_subject <= n_robots % robot
    x_measured_subject = state.robots{measured_subject}(1:2);
else % landmarks
    x_measured_subject = state.landmarks(state.landmarks(:, 1) == measured_subject, 2:3)';
end
x2x_measumed = Rot'*(x_measured_subject-x_subject);

y_hat = [norm(x2x_measumed);
    atan2(x2x_measumed(2),x2x_measumed(1))]; %[r; phi]
end

