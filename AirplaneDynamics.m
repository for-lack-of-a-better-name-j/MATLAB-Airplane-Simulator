function k = AirplaneDynamics(x,...
    fx, fy, fz, Tx, Ty, Tz, mass, Jx, Jy, Jz, Jxy)
% k = AirplaneDynamics(x,fx, fy, fz, Tx, Ty, Tz, mass,
%                                       Jx, Jy, Jz, Jxy)
%
%AirplaneDynamics calculates the runge kutta k variable for the airplane
% equations of motion give the current state x, the forces, and torques.
%k: the runge kutta k variable (k1, k2, k3, or k4)
%x: the state vector, it has the following 13 state variables:
% x=[u;v;w;p;q;r;pn;pu;pe;e0;e1;e2;e3]
% x(1): u (m/s) is the x-velocity of the airplane in the body frame
% x(2): v (m/s) is the y-velocity of the airplane in the body frame
% x(3): w (m/s) is the z-velocity of the airplane in the body frame
% x(4): p (rad/s) roll rate about the body frame x-axis
% x(5): q (rad/s) yaw rate about the body frame y-axis
% x(6): r (rad/s) pitch rate about the body frame z-axis
% x(7): pn (m) is the north position of the airplane in the inertial frame
% x(8): pu (m) is the up position of the airplane in the inertial frame
% x(9): pe (m) is the east position of the airplane in the inertial frame
% x(10): e0 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(11): e1 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(12): e2 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(13): e3 (rad) is a quaternion.  It is related to roll, pitch, and yaw
%fx: (N) force in the x-direction of the body frame
%fy: (N) force in the y-direction of the body frame
%fz: (N) force in the z-direction of the body frame 
%Tx: (N m) Torque about the x-axis in the body frame
%Ty: (N m) Torque about the y-axis in the body frame
%Tz: (N m) Torque about the z-axis in the body frame
%mass: (kg) mass of the airplane
%Jx: (kg m2) moment of inertia about the body x-axis
%Jy: (kg m2) moment of inertia about the body y-axis
%Jz: (kg m2) moment of inertia about the body z-axis
%Jxy: (kg m2) Product of inertia

 

  % after much wailing and gnashing of teeth,
  % I am going to assume that Jxy == Jxz in the book equations.
  Jxx = Jx;
  Jyy = Jy;
  Jzz = Jz;
  %Jxz = Jxy;
  Jxz = 0;
  Jyz = 0;
  %invJ = (1/(Jxx*Jyy*Jzz))*[Jyy*Jzz, 0, Jyy*Jxz; 0, Jxx*Jzz-Jxz^2, 0; Jyy*Jxz, 0, Jxx*Jyy];
  invJ = inv([Jxx, -Jxy, -Jxz; -Jxy, Jyy, -Jyz; -Jxz, -Jyz, Jzz]);
  m = mass;

  %extract the state variables
  u = x(1);%u (m/s) is the x-velocity of the airplane in the body frame
  v = x(2);%v (m/s) is the y-velocity of the airplane in the body frame
  w = x(3);%w (m/s) is the z-velocity of the airplane in the body frame
  p = x(4);%p (rad/s) roll rate in the body frame
  q = x(5);%q (rad/s) pitch rate in the body frame
  r = x(6);%r (rad/s) yaw rate in the body frame
  pn = x(7);% pn (m) is the north position of the airplane in the inertial frame
  pu = x(8);%pu (m) is the up position of the airplane in the inertial frame
  pe = x(9);%pe (m) is the east position of the airplane in the inertial frame
  e0 = x(10);%e0 (rad) is a quaternion. It is related to roll, pitch, and yaw
  e1 = x(11);%e1 (rad) is a quaternion. It is related to roll, pitch, and yaw
  e2 = x(12);%e2 (rad) is a quaternion. It is related to roll, pitch, and yaw
  e3 = x(13);%e3 (rad) is a quaternion. It is related to roll, pitch, and yaw


  %------------------------Write the Equations of motion here---------------%
  % body fixed 3d acceleration
  udot = r*v - q*w + (1/m)*fx;
  vdot = p*w-r*u + (1/m)*fy;
  wdot = q*u-p*v + (1/m)*fz;
  uvwdot = [udot;vdot;wdot];
  
  mult_mtx = [
    q*r*(Jyy-Jzz)-p*r*Jxy;
    p*r*(Jzz-Jxx)+q*r*Jxy;
    p*q*(Jxx-Jyy)+(p^2-q^2)*Jxy;
  ];
  Tmtx = [Tx; Ty; Tz];

  % 3d angular accelerations
  pqrdot = invJ*(mult_mtx + Tmtx)
  rotation_matrix = [
      e0^2 + e1^2 - e2^2 - e3^2, 2*(e1*e2-e0*e3), 2*(e0*e2 + e1*e3);
      2*(e0*e3 + e1*e2), e0^2-e1^2 + e2^2 - e3^2, 2*(e2*e3-e0*e1);
      2*(e1*e3-e0*e2), 2*(e0*e1 + e2*e3), e0^2-e1^2-e2^2 + e3^2
  ];
  uvw = [u;v;w];
  xyzIdot = rotation_matrix*uvw;
  quatmtx = [-e1, -e2, -e3; e0, -e3, e2; e3, e0, -e1; -e2, e1, e0];
  pqr = [p;q;r];

  % theta
  quat_dot = .5*quatmtx*pqr;
  k = [uvwdot;pqrdot;xyzIdot;quat_dot];


  %------------------------End of the Equations of motion-------------------%
