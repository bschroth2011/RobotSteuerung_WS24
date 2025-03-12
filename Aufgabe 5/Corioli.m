clear, close all;

robot = loadrobot('kukaIiwa14', 'DataFormat', 'column'); % Load Franka Panda robot

%konstant location
q = [0.3; -0.5; 0.3; -0.1; 0.7; -0.4; 0.5]; 

%different velocities
q_dot1 = [0.3; 0.3;0.3;0.3;0.3 ;0.3;0.3]; 
q_dot2 = [0.6; 0.6;0.6;0.6;0.6 ;0.6;0.6]; 
q_dot3 = [0.9; 0.9;0.9;0.9;0.9 ;0.9;0.9]; 


q_dot_array = [q_dot1, q_dot2, q_dot3];

%konstant acceleration
q_ddot = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5]; 

J = geometricJacobian(robot, q, 'iiwa_link_ee'); % Compute Jacobian for ee

% Compute Mass-Inertia Matrix
M = massMatrix(robot, q);

% Compute Euler Forces
euler_forces = M * q_ddot;


disp('Euler Forces:'), disp(euler_forces);

% Compute Coriolis Forces
for i = 1:3

end_effector_velocity = J * q_dot_array(:,i);
coriolis_forces = velocityProduct(robot, q, q_dot_array(:,i));

Coriolis_Vec(:,i) = coriolis_forces;



end

disp('Coriolis Forces v1 bis 3')
disp(Coriolis_Vec)

% disp('End-Effector Velocity (Linear + Angular):')
% disp(end_effector_velocity)