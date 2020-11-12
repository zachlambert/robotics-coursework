function config_dot_cmd = leg_config_dot_ik(config, vx_base, vy_base, omega)
phi = config(1);
theta1 = config(2);
theta2 = config(3);

vx = vx_base - omega*sin(phi)*(l1 + l3*cos(theta1 + theta2) + l2*cos(theta1));
vy = vy_base + omega*cos(phi)*(l1 + l3*cos(theta1 + theta2) + l2*cos(theta1));
  
Jinv = [
    -sin(phi)/(l1*sin(phi)^2 + l1*cos(phi)^2 + l3*cos(theta1 + theta2)*cos(phi)^2 + l3*cos(theta1 + theta2)*sin(phi)^2 + l2*cos(phi)^2*cos(theta1) + l2*cos(theta1)*sin(phi)^2),                                                                                            cos(phi)/(l1*sin(phi)^2 + l1*cos(phi)^2 + l3*cos(theta1 + theta2)*cos(phi)^2 + l3*cos(theta1 + theta2)*sin(phi)^2 + l2*cos(phi)^2*cos(theta1) + l2*cos(theta1)*sin(phi)^2),                                                                                                                             0;
    -(cos(theta1 + theta2)*cos(phi))/(l2*cos(theta1 + theta2)*cos(phi)^2*sin(theta1) - l2*sin(theta1 + theta2)*cos(phi)^2*cos(theta1) + l2*cos(theta1 + theta2)*sin(phi)^2*sin(theta1) - l2*sin(theta1 + theta2)*cos(theta1)*sin(phi)^2),                                  -(cos(theta1 + theta2)*sin(phi))/(l2*cos(theta1 + theta2)*cos(phi)^2*sin(theta1) - l2*sin(theta1 + theta2)*cos(phi)^2*cos(theta1) + l2*cos(theta1 + theta2)*sin(phi)^2*sin(theta1) - l2*sin(theta1 + theta2)*cos(theta1)*sin(phi)^2),                              sin(theta1 + theta2)/(l2*cos(theta1 + theta2)*sin(theta1) - l2*sin(theta1 + theta2)*cos(theta1));
    (cos(phi)*(l3*cos(theta1 + theta2) + l2*cos(theta1)))/(l2*l3*cos(theta1 + theta2)*sin(phi)^2*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(theta1)*sin(phi)^2 + l2*l3*cos(theta1 + theta2)*cos(phi)^2*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(phi)^2*cos(theta1)), (sin(phi)*(l3*cos(theta1 + theta2) + l2*cos(theta1)))/(l2*l3*cos(theta1 + theta2)*sin(phi)^2*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(theta1)*sin(phi)^2 + l2*l3*cos(theta1 + theta2)*cos(phi)^2*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(phi)^2*cos(theta1)), -(l3*sin(theta1 + theta2) + l2*sin(theta1))/(l2*l3*cos(theta1 + theta2)*sin(theta1) - l2*l3*sin(theta1 + theta2)*cos(theta1))
];

config_dot_cmd = Jinv*[vx;vy;0];

end