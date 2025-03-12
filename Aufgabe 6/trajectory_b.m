clear,close all;
%load Robot
robot = loadrobot('kukaIiwa14', 'DataFormat', 'column'); % Load KUKA LBR iiwa

% Number of joints in the robot
numJoints = numel(robot.homeConfiguration);

%IK-Object
ik = inverseKinematics('RigidBodyTree', robot);
initialGuess = robot.homeConfiguration;
ik.SolverParameters.MaxIterations = 5000;

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
 

% Loop through each pose -> für Berechnung Trapezlaufbahn -> wichtig für Timesscaling
for i = 1:size(poses, 1)

    % Extract position and orientation
    position = poses(i, 1:3);       % X, Y, Z
    orientation = poses(i, 4:6);    % Roll, Pitch, Yaw

    % Create end-effector pose using position and orientation
    endEffectorPose = trvec2tform(position) * eul2tform(orientation);

    % Solve Ik
    [configSol, solInfo] = ik('iiwa_link_ee', endEffectorPose, ones(1,6), initialGuess);

    wayPoints_rad(:,i) = configSol; % Joint-Konfig

    initialGuess = configSol; %clean trajectory

    %Check für Singularities
    if solInfo.Iterations >= 200
        disp("error");
        disp(i);
        disp(solInfo.Iterations);
        break;    
    end
end


[q,qd,qdd,tSamples,pp] = trapveltraj(wayPoints_rad,sample,EndTime=2);


%für Aufgabe 6b)
tInterval = [0 tSamples(end)];

%start and endpoint (t-Matrix)
T0 = trvec2tform(poses(1, 1:3)) * eul2tform(poses(1, 4:6));
TF = trvec2tform(poses(7, 1:3)) * eul2tform(poses(7, 4:6));

%[tforms,vel,acc] = transformtraj(T0,TF,tInterval,tSamples);

s = zeros (3,sample); %Vektor for Timescaling


%Normierung weil Timescaling werte von 0 bis 1 braucht
x = (tSamples - tSamples(1)) / (tSamples(end) - tSamples(1));
s(1,:) = x;   % Normierte Position
s(2,:) = qd(1,:) / max(qd(1,:));  % Normierte Geschwindigkeit
s(3,:) = qdd(1,:) / max(qdd(1,:));  % Normierte Beschleunigung


[tforms,vel,acc] = transformtraj(T0,TF,tInterval,tSamples,'TimeScaling',s);

rotations = tform2quat(tforms);
translations = tform2trvec(tforms);

for i = 1:sample
    endEffectorPose = tforms(:,:,i); 

    % IK-Solver
    [configSol, solInfo] = ik('iiwa_link_ee', endEffectorPose, ones(1,6), initialGuess);

 
    

    %Jacobi_matrix für geschw. und beschl.
    J = geometricJacobian(robot, configSol, 'iiwa_link_ee');

    if i > 1
        J_prev = geometricJacobian(robot, q_gelenk(:,i-1), 'iiwa_link_ee');
        J_dot = (J - J_prev) / (tSamples(i) - tSamples(i-1));
    else
        J_dot = zeros(size(J));
    end

    %Values from transformtraj
    v_rot = vel(1:6, i);
    a_rot = acc(1:6, i);

    %Joint-Values for position, velocitie and acceleration
    q_gelenk(:,i) = configSol;
    qd_gelenk(:,i) = pinv(J) * v_rot;
    qdd_gelenk(:,i) = pinv(J) * (a_rot - J_dot * qd_gelenk(:,i));

    initialGuess = configSol; %clean trajectory

    if solInfo.Iterations >= 200
        disp("error");
        disp(i);
        disp(solInfo.Iterations);
        break;    
    end
end

%plot for Joint-konfig, -velocitie and acceleration 
figure;
for i = 1:numJoints
    subplot(3, numJoints, i);
    plot(tSamples, q_gelenk(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['q', num2str(i), ' (rad)']);
    title(['Joint ', num2str(i), ' Position']);
    grid on;

    subplot(3, numJoints, i + numJoints);
    plot(tSamples, qd_gelenk(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['qd', num2str(i), ' (rad/s)']);
    title(['Joint ', num2str(i), ' Velocity']);
    grid on;

    subplot(3, numJoints, i + 2*numJoints);
    plot(tSamples, qdd_gelenk(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel(['qdd', num2str(i), ' (rad/s²)']);
    title(['Joint ', num2str(i), ' Acceleration']);
    grid on;
end

%plot Trajecory in 3d
figure;
plot3(translations(:,1), translations(:,2), translations(:,3), 'b-', 'LineWidth', 2);
hold on;
grid on;
xlim([-1 1]); ylim([-1 1]); zlim([0 1]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Endeffektor-Trajektorie im 3D-Raum');
view(3); % 3D-Ansicht aktivieren

% first frame of Robot
figure;
ax = show(robot, q_gelenk(:,1)); % Erstes Frame anzeigen
hold on;
xlim([-1 1]); ylim([-1 1]); zlim([0 1]); % Achsenbegrenzungen setzen

% Animationsschleife
for i = 1:sample
    show(robot, q_gelenk(:,i), 'PreservePlot', false, 'Parent', ax);
    pause(0.05); % Zeitverzögerung für flüssige Animation
end