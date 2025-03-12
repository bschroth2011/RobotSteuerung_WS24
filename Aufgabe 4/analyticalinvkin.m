clear,close all;

% Load the robot model
robot = loadrobot('kukaIiwa14', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
aik = analyticalInverseKinematics(robot); %generierung vom AIK-Objekt
opts = showdetails(aik); %Anzeige kinematischer Gruppen

aik.KinematicGroup = opts(2).KinematicGroup; %auswahl kinemat. Gruppe 

generateIKFunction(aik,'robotIK'); %generierung aik-Solver-Funktion

ik = inverseKinematics('RigidBodyTree', robot); %ik objekt für 
initialGuess = robot.homeConfiguration;

pose_ee = [0.2, 0.6, 0.4, pi/2, 0, 0];
position = pose_ee(1, 1:3);       % X, Y, Z
orientation = pose_ee(1, 4:6);    % Roll, Pitch, Yaw

% Create ee pose using position and orientation
endEffectorPose = trvec2tform(position) * eul2tform(orientation);

% Solve Inverse Kinematics
[configSol, solInfo] = ik('iiwa_link_ee', endEffectorPose, ones(1,6), initialGuess);

pose_ee_rad = configSol;
%pose_ee_rad = [0.455910553666195	0.936861920078818	1.04083132567303	-1.78104425913282	-0.403462863193918	-0.622320199696265	-0.461517909195206];
%T = trvec2tform(pose_ee(1:3)) * eul2tform(pose_ee(4:6));

% pose = randomConfiguration(robot);


% ab hier Anwendung der AIK
expConfig = pose_ee_rad; %aus ik generiert

eeBodyName = aik.KinematicGroup.EndEffectorBodyName;
baseName = aik.KinematicGroup.BaseName;

expEEPose = getTransform(robot,expConfig,eeBodyName,baseName); %nimmt nur Gelenkkonfigs -> Funktion erstellt aus Gelenkwinkel T-Matrix
ikConfig = robotIK(expEEPose,true); %nur gültige konfigurationen -> gibt nur 6 Winkel aus (analyticalInverseKinematics macht nur 6 Winkel)

%Plot -> gibt anzahl der Lösungen von AIK aus
eeWorldPose = getTransform(robot,expConfig,eeBodyName);

generatedConfig = repmat(expConfig, size(ikConfig,1), 1);
generatedConfig(:,aik.KinematicGroupConfigIdx) = ikConfig;

for i = 1:size(ikConfig,1)
    figure;
    ax = show(robot,generatedConfig(i,:));
    hold all;
    plotTransforms(tform2trvec(eeWorldPose),tform2quat(eeWorldPose),'Parent',ax);
    title(['Solution ' num2str(i)]);
end


