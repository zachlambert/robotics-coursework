syms theta1 theta2 theta3
syms l1 l2 l3
syms vx vy

% Velocity is given as the desired velocity of the leg origin
% Expressed in the leg frame, with the x-axis pointing
% radially outward.

% The leg end effector must have an equal velocity
% in the opposite direction to push the leg origin.

% Inverse kinematics gives joint velocities required
% for this (opposite) velocity

r = l1 + l2*cos(theta2) + l3*cos(theta2 + theta3);
x = r*cos(theta1);
y = r*sin(theta1);
z = -l2*sin(theta2) - l3*sin(theta2 + theta3);

state = [x; y; z];
J = [diff(state, theta1) diff(state, theta2) diff(state, theta3) ];

%[theta1_dot; theta2_dot; theta3_dot] = -inv(J)*[vx;vy;0];
display(-inv(J)*[vx;vy;0]);