clear all

q = [7;6;5;4];
e = q/norm(q);
x = [3;2;1;5;4;3;6;5;4;e(1);e(2);e(3);e(4)];

fx = 3; fy = 2; fz = 7; Tx = 6; Ty = 2; Tz = 3.1; 
mass = 18.1; Jx = 2.3; Jy = 1.6; Jz = 6.6; Jxy = 0.1;

k = AirplaneDynamics(x, fx, fy, fz, Tx, Ty, Tz, mass, Jx, Jy, Jz, Jxy)'

%you should get
% k = [2.1657    -3.8895    2.3867  -22.3515   40.9155    2.7273    2.0476    2.7619    1.4762 -2.7617 1.5145 1.3363 0.8909]

 
