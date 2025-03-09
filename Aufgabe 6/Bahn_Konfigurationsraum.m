clc; clear; close all;

%% **1. Definition der Gelenk-Start- und Zielkonfiguration**
q_start = [0; -pi/4; 0; -pi/2; 0; pi/3; 0];  % Startwinkel in Rad
q_end   = [pi/4; 0; -pi/4; -pi/3; pi/4; -pi/4; pi/6];  % Zielwinkel

num_samples = 100;  % Anzahl der Trajektorienpunkte
total_time = 5;  % Gesamtzeit in Sekunden
time_vector = linspace(0, total_time, num_samples); % Zeitvektor

%% **2. Trapezförmige Trajektorie im Gelenkraum berechnen**
[q, qd, qdd] = trapveltraj([q_start, q_end], num_samples);

%% **3. Ergebnisse im Gelenkraum plotten**
figure;
subplot(3,1,1), plot(time_vector, q'), title('Gelenkwinkel über Zeit'), xlabel('Zeit [s]'), ylabel('Winkel [rad]');
subplot(3,1,2), plot(time_vector, qd'), title('Gelenkgeschwindigkeit über Zeit'), xlabel('Zeit [s]'), ylabel('Winkelgeschw. [rad/s]');
subplot(3,1,3), plot(time_vector, qdd'), title('Gelenkbeschleunigung über Zeit'), xlabel('Zeit [s]'), ylabel('Winkelbeschl. [rad/s²]');

%% **4. KUKA iiwa Modell laden (Peter Corke Toolbox oder eigene Kinematik)**
robot = loadrobot('kukaIiwa14', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

%% **5. Vorwärtskinematik für die Trajektorie berechnen (Arbeitsraum)**
tcp_positions = zeros(3, num_samples);

for i = 1:num_samples
    T = getTransform(robot, q(:, i), 'tool0');  % Homogene Transformationsmatrix
    tcp_positions(:, i) = T(1:3, 4);  % TCP-Position speichern
end

%% **6. Trajektorie im Arbeitsraum mit transformtraj generieren**
T_start = getTransform(robot, q_start, 'tool0');
T_end = getTransform(robot, q_end, 'tool0');

[T, Td, Tdd] = transformtraj(T_start, T_end, [0 total_time], time_vector);

%% **7. Ergebnisse im Arbeitsraum plotten**
figure;
plot3(tcp_positions(1, :), tcp_positions(2, :), tcp_positions(3, :), 'r', 'LineWidth', 2);
hold on;
plot3(T(1,:), T(2,:), T(3,:), 'b--', 'LineWidth', 1.5);
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]');
title('Trajektorie des TCP im Arbeitsraum');
legend('Vorwärtskinematik (Joint Space)', 'transformtraj (Cartesian Space)');
grid on; view(3);

disp('Simulation abgeschlossen.');
