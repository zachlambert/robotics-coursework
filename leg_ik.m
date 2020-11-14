function config_dot_cmd = leg_ik(config, vx, vy, l1, l2, l3)
% velocity inverse kinematics
% For a given velocity of the leg origin, expressed in the leg
% frame (X radially outward), give the joint velocities required
% for the leg to push itself with this velocity.

theta1 = config(1);
theta2 = config(2);
theta3 = config(3);

% Coped from leg_ik_symb.m output
config_dot_cmd = [
    (vx*sin(theta1))/(l1*cos(theta1)^2 + l1*sin(theta1)^2 + l3*cos(theta2 + theta3)*cos(theta1)^2 + l3*cos(theta2 + theta3)*sin(theta1)^2 + l2*cos(theta1)^2*cos(theta2) + l2*cos(theta2)*sin(theta1)^2) - (vy*cos(theta1))/(l1*cos(theta1)^2 + l1*sin(theta1)^2 + l3*cos(theta2 + theta3)*cos(theta1)^2 + l3*cos(theta2 + theta3)*sin(theta1)^2 + l2*cos(theta1)^2*cos(theta2) + l2*cos(theta2)*sin(theta1)^2);
    (vx*cos(theta2 + theta3)*cos(theta1))/(l2*cos(theta2 + theta3)*cos(theta1)^2*sin(theta2) - l2*sin(theta2 + theta3)*cos(theta1)^2*cos(theta2) + l2*cos(theta2 + theta3)*sin(theta1)^2*sin(theta2) - l2*sin(theta2 + theta3)*cos(theta2)*sin(theta1)^2) + (vy*cos(theta2 + theta3)*sin(theta1))/(l2*cos(theta2 + theta3)*cos(theta1)^2*sin(theta2) - l2*sin(theta2 + theta3)*cos(theta1)^2*cos(theta2) + l2*cos(theta2 + theta3)*sin(theta1)^2*sin(theta2) - l2*sin(theta2 + theta3)*cos(theta2)*sin(theta1)^2);
    -(vx*cos(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2)))/(l2*l3*cos(theta2 + theta3)*cos(theta1)^2*sin(theta2) - l2*l3*sin(theta2 + theta3)*cos(theta1)^2*cos(theta2) + l2*l3*cos(theta2 + theta3)*sin(theta1)^2*sin(theta2) - l2*l3*sin(theta2 + theta3)*cos(theta2)*sin(theta1)^2) - (vy*sin(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2)))/(l2*l3*cos(theta2 + theta3)*cos(theta1)^2*sin(theta2) - l2*l3*sin(theta2 + theta3)*cos(theta1)^2*cos(theta2) + l2*l3*cos(theta2 + theta3)*sin(theta1)^2*sin(theta2) - l2*l3*sin(theta2 + theta3)*cos(theta2)*sin(theta1)^2)
];

end