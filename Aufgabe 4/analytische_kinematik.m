clc; clear; close all;

%% **1. KUKA LBR iiwa 14 Modell laden**
robot = loadrobot('kukaIiwa14', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

%% **2. DH-Parameter für analytische IK**
% Parameter (Längen gemäß Transformationen)
d = [0.1575, 0.2025, 0.2045, 0.2155, 0.1845, 0.2155, 0.0810];
alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];

%% **3. Zielpose definieren (X, Y, Z, Roll, Pitch, Yaw)**
% Beispielziel für den Endeffektor (X, Y, Z, Rotation)
targetPose = [0.3, 0.2, 0.5, pi/4, -pi/4, pi/2];

% Position extrahieren
Px = targetPose(1);
Py = targetPose(2);
Pz = targetPose(3);

% Rotation extrahieren
roll = targetPose(4);
pitch = targetPose(5);
yaw = targetPose(6);

%% **4. Berechnung der analytischen IK**
% **Schritt 1: Gelenk 1 berechnen (Rotation um Z-Achse der Basis)**
q1 = atan2(Py, Px);

% **Schritt 2: Gelenke 2 & 3 berechnen (Ebene XY)**
% Berechnung der Projektion der Position in die X-Z-Ebene
r = sqrt(Px^2 + Py^2);
s = Pz - d(1);

L1 = d(2);
L2 = d(3);

% Kosinussatz zur Berechnung von q3
cos_q3 = (r^2 + s^2 - L1^2 - L2^2) / (2 * L1 * L2);

% Begrenzung von cos(q3) auf den gültigen Bereich [-1,1]
cos_q3 = max(min(cos_q3, 1), -1); 

% Berechnung von q3 mit atan2
q3 = atan2(sqrt(1 - cos_q3^2), cos_q3); % Zwei Lösungen möglich!


% Berechnung von q2
q2 = atan2(s, r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3));

% **Schritt 3: Gelenke 4–6 für die Orientierung berechnen**
R06 = eul2rotm([roll, pitch, yaw], 'ZYX'); % Zielorientierung

% Berechnung der Rotationen für q4, q5, q6 (inverse Rotation)
R03 = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1] * ...
      [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1] * ...
      [cos(q3) -sin(q3) 0; sin(q3) cos(q3) 0; 0 0 1];

R36 = R03' * R06; % Berechnung der Orientierung der letzten drei Gelenke

% Berechnung von q4, q5, q6 aus der Rotationsmatrix
q4 = atan2(R36(2,3), R36(1,3));
q5 = atan2(sqrt(1 - R36(3,3)^2), R36(3,3));
q6 = atan2(R36(3,2), -R36(3,1));

% **Schritt 4: Gelenk 7 fixieren**
q7 = 0; % Hier könnte eine Bedingung definiert werden

% Ergebnis ausgeben
q = [q1, q2, q3, q4, q5, q6, q7];
fprintf('Gelenkwinkel in Grad:\n');
disp(rad2deg(q));

%% **5. Roboter mit berechneter IK-Lösung visualisieren**
% 1️⃣ Korrekte Initialisierung: Home-Konfiguration holen
configSol = homeConfiguration(robot); % Gibt ein struct-Array zurück

% 2️⃣ Sicherstellen, dass configSol wirklich ein struct-Array ist
if ~isstruct(configSol)
    error('homeConfiguration(robot) gibt kein struct zurück!');
end

% 3️⃣ Gelenkwinkel korrekt setzen
for i = 1:length(q)
    configSol(i).JointPosition = q(i); % Jetzt sollte die Zuweisung klappen!
end

% 4️⃣ Roboter mit der neuen Gelenkkonfiguration anzeigen
figure;
show(robot, configSol, 'Frames', 'off', 'PreservePlot', false);
title('Analytische IK Lösung');
view(3);
axis auto;
grid on;
