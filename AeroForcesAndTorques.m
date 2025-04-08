function [fxD,fyL,fzbeta,TxAero,Ty,Tz] = ...
    AeroForcesAndTorques(x,Va,alpha,beta,...
    delta_e,delta_a,delta_r,consts)
%[fxD,fyL,fzbeta,TxAero,TyAero,TzAero] = ...
%    AeroForcesAndTorques(x,Va,alpha,beta,...
%    delta_e,delta_a,delta_r,consts)
%
%OUTPUTS:
% fxD: (N) aerodynamic force in the body-fixed x-direction
% fyL: (N) aerodynamic force in the body-fixed y-direction
% fzbeta: (N) aerodynamic force in the body-fixed z-direction
% TxAero: (N m) aerodynamic torque around the body-fixed x-axis
% Ty: (N m) aerodynamic torque around the body-fixed y-axis
% Tz: (N m) aerodynamic torque around the body-fixed z-axis
%
%INPUTS:
% x: a vector of 13 state variables x=[u;v;w;p;q;r;xI;yI;zI;e0;e1;e2;e3]
% Va: (m/s) airspeed
% alpha: (rad) angle of attack
% beta: (rad) side-slip angle
% delta_e: (-1 to 1) elevator command
% delta_a: (-1 to 1) aileron command
% delta_r: (-1 to 1) rudder command
% consts: a struct with at least the following constants
%    consts.rho: (kg/m3) air density
%    consts.A: (m2) wing area
%    consts.CL_alpha: (unitless) lift coefficient
%    consts.CL_delta_e: (unitless) lift coefficient
%    consts.CD_alpha: (unitless) drag coefficient
%    consts.CD_delta_e: (unitless) drag coefficient
%    consts.Cz_beta: (unitless) side-slip coefficient
%    consts.Cz_delta_r: (unitless) side-slip coefficient
%    consts.c: (m) wing cord
%    consts.CTz_alpha: (unitless) pitch torque coefficient
%    consts.CTz_delta_e: (unitless) pitch torque coefficient
%    consts.br: (N m s) pitch torque aerodynamic resistance
%    consts.S: (m) wing span
%    consts.C_delta_a: (unitless) roll torque coefficient
%    consts.bp: (N m s) roll torque aerodynamic resistance
%    consts.CTy_beta: (unitless) yaw torque coefficient
%    consts.CTy_delta_r: (unitless) yaw torque coefficient
%    consts.bq: (N m s) yaw torque aerodynamic resistance
%
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


  c = consts; % reassigning, I got tired of writing that in the last one
  f_aero = .5*c.rho*Va^2*c.A;
  fxD = f_aero*(-c.CD_alpha*cos(alpha)-c.CD_delta_e*delta_e);
  fyL = f_aero*(c.CL_alpha*alpha+c.CL_delta_e*delta_e);
  fzbeta = f_aero*(-c.Cz_beta*beta-c.Cz_delta_r*delta_r);
  TxAero = f_aero*c.S*c.C_delta_a*delta_a-c.bp*p;
  Ty = f_aero*c.c*(-c.CTy_beta*beta-c.CTy_delta_r*delta_r)-c.bq*q;
  Tz = f_aero*c.c*(-c.CTz_alpha*alpha-c.CTz_delta_e*delta_e)-c.br*r;
    
    
