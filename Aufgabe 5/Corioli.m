robot = loadrobot('kukaIiwa14', 'DataFormat', 'column'); % Load Franka Panda robot

q = [0.3; -0.5; 0.3; -0.1; 0.7; -0.4; 0.5]; 
q_dot = [1.4834; 1.4834;1.7452;1.3089;2.2688 ;2.356;2.356]; 
q_ddot = [0.5; 0.2; -0.3; 0.1; 0.4; -0.2; 0]; 

J = geometricJacobian(robot, q, 'iiwa_link_ee'); % Compute Jacobian for end-effector
end_effector_velocity = J * q_dot;

% Compute Mass-Inertia Matrix
M = massMatrix(robot, q);

% Compute Euler Forces
euler_forces = M * q_ddot;

disp('End-Effector Velocity (Linear + Angular):')
disp(end_effector_velocity)

coriolis_forces = velocityProduct(robot, q, q_dot);

disp('Coriolis Forces')
disp(coriolis_forces)

disp('Euler Forces:'), disp(euler_forces)