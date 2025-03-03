% Load the KUKA LBR iiwa 14 robot
robot = loadrobot('kukaIiwa14', 'DataFormat', 'column', 'Gravity', [0 0 -9.81]);

% Define two configurations: Vertical and Horizontal
config_vertical = [0; 0; 0; 0; 0; 0; 0];       % Arm is vertical (Home position)
config_horizontal = [0; -pi/2; 0; 0; 0; 0; 0]; % Arm is extended horizontally
config_vertical_down = [0; -pi/2; -pi/2; 0; 0; 0; 0];
% Compute gravitational torques for both configurations
tau_vertical = gravityTorque(robot, config_vertical);
tau_horizontal = gravityTorque(robot, config_horizontal);

% Store torques in a matrix
torques = [tau_vertical'; tau_horizontal'];

% Find min and max torques for each joint
min_torques = min(torques);
max_torques = max(torques);

% Display results
disp('Gravitational torques for vertical configuration:');
disp(tau_vertical);

disp('Gravitational torques for horizontal configuration:');
disp(tau_horizontal);

disp('Minimum torques across configurations:');
disp(min_torques);

disp('Maximum torques across configurations:');
disp(max_torques);

% Visualize both configurations
figure;
subplot(1, 3, 1);
show(robot, config_vertical);
title('Vertical Configuration');

subplot(1, 3, 2);
show(robot, config_horizontal);
title('Horizontal Configuration');

subplot(1, 3, 3);
show(robot, config_vertical_down);
title('vertical_down Configuration');
