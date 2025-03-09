clc; clear; close all;

% KUKA iiwa 14 Roboter aus MATLAB laden
robot = loadrobot('kukaIiwa14', 'DataFormat', 'row');

% Gelenkpositionen und DH-Parameter extrahieren
fprintf('Gelenk-Transformationen:\n');
for i = 1:length(robot.Bodies)
    body = robot.Bodies{i};
    joint = body.Joint;
    
    % Transformation vom Parent-Link zum aktuellen Gelenk
    T = joint.JointToParentTransform;
    
    % DH-ähnliche Parameter extrahieren
    fprintf('Gelenk %d:\n', i);
    disp(T);
end
