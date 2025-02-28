% Load the robot model
robot = loadrobot('kukaIiwa14', 'DataFormat', 'column');

% Define different joint configurations
config1 = [0; 0; 0; 0; 0; 0; 0];       % Vertical position
config2 = [0; -pi/2; 0; 0; 0; 0; 0];   % Horizontal position
config3 = [pi/4; pi/4; -pi/4; 0; pi/6; 0; 0]; % Intermediate position

% Compute mass matrices for each configuration
M1 = massMatrix(robot, config1);
M2 = massMatrix(robot, config2);
M3 = massMatrix(robot, config3);

% Display results
disp('Mass Matrix at Configuration 1 (Vertical):');
disp(M1);

disp('Mass Matrix at Configuration 2 (Horizontal):');
disp(M2);

disp('Mass Matrix at Configuration 3 (Intermediate):');
disp(M3);
