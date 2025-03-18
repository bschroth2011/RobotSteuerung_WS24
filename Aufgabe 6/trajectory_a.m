clear,close all;

robot = loadrobot('kukaIiwa14', 'DataFormat', 'column'); % Load KUKA LBR iiwa

% Number of joints in the robot(7)
numJoints = numel(robot.homeConfiguration);

%IK-Object
ik = inverseKinematics('RigidBodyTree', robot);
initialGuess = robot.homeConfiguration;
ik.SolverParameters.MaxIterations = 5000;


q_gelenk = zeros(1,7);

start_pose = [0.0, -0.8, 0.6, -pi/2, 0, 0]; %[X, Y, Z, Roll, Pitch, Yaw] (angles in radians)

end_pose = [-0.6, -0.1, 0.5, -pi, 0, 0];

%Amount of samples (steps) between start and end
sample = 300;

%beginn and end with waypoints (5)
wayPoints = [0.0,-0.1,-0.3,-0.5,-0.6,-0.6,-0.6;    
            -0.8,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1;
             0.6,0.6,0.6,0.6,0.6,0.5,0.5;
            -pi/2,-pi/2,-pi/1.6,-pi/1.3,-pi,-pi,-pi;
             0.0,0.0,0.0,0.0,0.0,0.0,0.0;
             0.0,0.0,0.0,0.0,0.0,0.0,0.0;
             ];

poses = wayPoints.';
 

% Loop through each pose
for i = 1:size(poses, 1)

    % Extract position and orientation
    position = poses(i, 1:3);       % X, Y, Z
    orientation = poses(i, 4:6);    % Roll, Pitch, Yaw

    % Create ee pose using position and orientation
    endEffectorPose = trvec2tform(position) * eul2tform(orientation);

    % Solve Ik

    [configSol, solInfo] = ik('iiwa_link_ee', endEffectorPose, ones(1,6), initialGuess);


    wayPoints_rad(:,i) = configSol; % Joint-Konfig 

    initialGuess = configSol; %clean trajectory -> (nimmt vorherige Konfig als Referenz)

    %Check für Singularities
    if solInfo.Iterations >= 200
        disp("error");
        disp(i);
        disp(solInfo.Iterations);
        break;    
    end
end



%für Aufgabe 6a) -> Trapezlaufbahn von Punkt zu Punkt
[q,qd,qdd,tSamples,pp] = trapveltraj(wayPoints_rad,sample);

%3d-Trajektorie -> Variable für Fahrlinie des ee
for i = 1:sample
effpos = getTransform(robot, q(:,i), 'iiwa_link_ee');
trajec_3e(i,1:3) = effpos(1:3,4);
end

%plot Trajecory in 3d
figure;
plot3(trajec_3e(:,1), trajec_3e(:,2), trajec_3e(:,3), 'b-', 'LineWidth', 2);
hold on;
grid on;
xlim([-1 1]); ylim([-1 1]); zlim([0 1]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Endeffektor-Trajektorie im 3D-Raum');
view(3); 




%plot for Joint-konfig, -velocitie and acceleration
figure;
for i = 1:numJoints
    subplot(3, numJoints, i);
    plot(tSamples, q(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['q', num2str(i), ' (rad)']);
    title(['Joint ', num2str(i), ' Position']);
    grid on;

    subplot(3, numJoints, i + numJoints);
    plot(tSamples, qd(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['qd', num2str(i), ' (rad/s)']);
    title(['Joint ', num2str(i), ' Velocity']);
    grid on;

    subplot(3, numJoints, i + 2*numJoints);
    plot(tSamples, qdd(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['qdd', num2str(i), ' (rad/s²)']);
    title(['Joint ', num2str(i), ' Acceleration']);
    grid on;
end


% first frame of Robot
figure;
ax = show(robot, q(:,1)); 
hold on;
xlim([-1 1]); ylim([-1 1]); zlim([0 1]); % Limits

% Animationsschleife
for i = 1:size(q, 2)
    show(robot, q(:,i), 'PreservePlot', false, 'Parent', ax);
    pause(0.00001); % Delay for clean trajectory-display
end
