close all;

% Load the robot model
robot = loadrobot('kukaIiwa14', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

% Create Inverse Kinematics object
ik = inverseKinematics('RigidBodyTree', robot);

% Define end-effector poses [X, Y, Z, Roll, Pitch, Yaw] (angles in radians)
poses = [0.0, 0.0, 1.4, 0, -pi/2, 0;    % Pose 1
         0.0, -0.0, 1.3, 0, -pi/2, 0;    % Pose 2
         0.2, 0.6, 0.4, pi/2, 0, 0];    % Pose 3
%poses = Pose;


wayPoints = [0.0,-0.6,-0.6,-0.6,-0.6,-0.6,-0.6;    
            -0.8,-0.6,-0.5,-0.4,-0.3,-0.2,0.0;
             0.6,0.6,0.6,0.5,0.5,0.5,0.5;
            -pi/2,-pi,-pi,-pi,-pi,-pi,-pi;
             0.0,0.0,0.0,0.0,0.0,0.0,0.0;
             0.0,0.0,0.0,0.0,0.0,0.0,0.0;
             ];

% poses = wayPoints.';
 


% Preallocate for iteration count and computation time
iterationCounts = zeros(size(poses, 1), 1);
computationTimes = zeros(size(poses, 1), 1);

% Initial guess for joint angles
initialGuess = robot.homeConfiguration;

ik.SolverParameters.MaxIterations = 5000;


% Loop through each pose
for i = 1:size(poses, 1)
    % Extract position and orientation
    position = poses(i, 1:3);       % X, Y, Z
    orientation = poses(i, 4:6);    % Roll, Pitch, Yaw

    % Create end-effector pose using position and orientation
    endEffectorPose = trvec2tform(position) * eul2tform(orientation);

    % Solve Inverse Kinematics
    tic;
    [configSol, solInfo] = ik('iiwa_link_ee', endEffectorPose, ones(1,6), initialGuess);
    computationTimes(i) = toc;

    angle_rad(i,:) = configSol;
    angle_degree(i,:) = rad2deg(configSol);

    

    % Store Iterations
    iterationCounts(i) = solInfo.Iterations;

    % Display Iteration Count
    fprintf('Pose %d: Iterations = %d, Time = %.4f sec\n', i, solInfo.Iterations, computationTimes(i));

    % Plot Robot Configuration
    figure;
    show(robot, configSol, 'Frames', 'off', 'PreservePlot', false);
    title(['Configuration for Pose ', num2str(i)]);
    view(3);
    axis auto;
    grid on;
end

