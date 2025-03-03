robot = loadrobot('kukaIiwa14', 'DataFormat', 'column'); % Load KUKA LBR iiwa
q = [0.1; -0.5; 0.3; -0.1; 0.2; -0.4; 0.5];   % Joint positions

F_ext = [0; 5 ; 0; 0; 0; 0]; % Example: 5N force in X-direction at the end-effector

J = geometricJacobian(robot, q, 'iiwa_link_ee'); % Get Jacobian for end-effector

tau_ext = J' * F_ext; % Compute the joint torques required to counteract F_ext

disp('Joint torques due to external force:')
disp(tau_ext)

show(robot, q, 'Frames', 'on');