syms phi theta1 theta2 t phi_dot theta1_dot theta2_dot
syms l1 l2 l3
syms vx_base vy_base vphi_base
syms vx vy

r = l1 + l2*cos(theta1) + l3*cos(theta1 + theta2);
x = r*cos(phi);
y = r*sin(phi);
z = -l2*sin(theta1) - l3*sin(theta1 + theta2);

state = [x; y; z];
J = [diff(state, phi) diff(state, theta1) diff(state, theta2) ];
Jinv = inv(J);

vx = vx_base - r*vphi_base*sin(phi)
vy = vy_base + r*vphi_base*cos(phi)
Jinv