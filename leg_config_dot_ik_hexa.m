function config_dot_cmd = leg_config_dot_ik(config, vx_base, vy_base, omega, alfa) %alfa is the offset angle, it is k/3*pi, where k is the number of the leg
phi = config(1);
theta1 = config(2);
theta2 = config(3);

vx = vx_base - omega*sin(phi)*(l1 + l3*cos(theta1 + theta2) + l2*cos(theta1));
vy = vy_base + omega*cos(phi)*(l1 + l3*cos(theta1 + theta2) + l2*cos(theta1));
  
Jinv = [                                                                          -sin(alpha + phi)/(l1*cos(alpha + phi)^2 + l1*sin(alpha + phi)^2 + l3*cos(alpha + phi)^2*cos(theta1 + theta2) + l3*sin(alpha + phi)^2*cos(theta1 + theta2) + l2*cos(alpha + phi)^2*cos(theta1) + l2*sin(alpha + phi)^2*cos(theta1)),                                                                            cos(alpha + phi)/(l1*cos(alpha + phi)^2 + l1*sin(alpha + phi)^2 + l3*cos(alpha + phi)^2*cos(theta1 + theta2) + l3*sin(alpha + phi)^2*cos(theta1 + theta2) + l2*cos(alpha + phi)^2*cos(theta1) + l2*sin(alpha + phi)^2*cos(theta1)),                                                                                                                             0;
                                 -(cos(alpha + phi)*cos(theta1 + theta2))/(l2*cos(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*cos(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1) + l2*sin(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*sin(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1)),                                  -(sin(alpha + phi)*cos(theta1 + theta2))/(l2*cos(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*cos(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1) + l2*sin(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*sin(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1)),                              sin(theta1 + theta2)/(l2*cos(theta1 + theta2)*sin(theta1) - l2*sin(theta1 + theta2)*cos(theta1));
(cos(alpha + phi)*(l3*cos(theta1 + theta2) + l2*cos(theta1)))/(l2*l3*cos(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*l3*cos(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1) + l2*l3*sin(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*l3*sin(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1)), (sin(alpha + phi)*(l3*cos(theta1 + theta2) + l2*cos(theta1)))/(l2*l3*cos(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*l3*cos(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1) + l2*l3*sin(alpha + phi)^2*cos(theta1 + theta2)*sin(theta1) - l2*l3*sin(alpha + phi)^2*sin(theta1 + theta2)*cos(theta1)), -(l3*sin(theta1 + theta2) + l2*sin(theta1))/(l2*l3*cos(theta1 + theta2)*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(theta1))];
 

config_dot_cmd = Jinv*[vx;vy;0];

end