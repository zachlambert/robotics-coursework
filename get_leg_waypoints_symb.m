% Fit x(t) to:
% x(t) = k4*t^4 + k3*t^3 + k2*t_2 + k1*t_1 + k0
% x(0) = x0
% x(T) = xT
% x_dot(0) = 0
% x_dot(T) = 0
% x(T/2) = xMid
syms k0 k1 k2 k3 k4 T x x_dot t x0 xT xMid

x = k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0;
x_dot = diff(x, t);

eq1 = [subs(x, t, 0) == x0];
eq2 = [subs(x, t, T) == xT];
eq3 = [subs(x_dot, t, 0) == 0];
eq4 = [subs(x_dot, t, T) == 0];
eq5 = [subs(x, t, T/2) == xMid];

eqs = [eq1, eq2, eq3, eq4, eq5];
S = solve(eqs, k0, k1, k2, k3, k4);
k0 = S.k0
k1 = S.k1
k2 = S.k2
k3 = S.k3
k4 = S.k4