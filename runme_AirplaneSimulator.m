close all
clear all
clc

xNorth = 0; %(m) North position in an inertial reference frame
yUp = 0; %(m) Up position in an inertial reference frame
zEast = 0; %(m) East position in an inertial reference frame
e0 = 1; %Quaternion
e1 = 0; %Quaternion
e2 = 0; %Quaternion
e3 = 0; %Quaternion

% FT Corsair constants
parameters = struct;

parameters.mass = 1; % kg
parameters.Jxx = .12; % kg/m^2
parameters.Jyy = .18; % kg/m^2
parameters.Jzz = .15; % kg/m^2
parameters.Jxy = .015;
parameters.Jxz = 0;
parameters.Jyz = 0;
parameters.S = 1.5; % m
parameters.c = .3; % m
parameters.A = .25;% m^2
parameters.CL_alpha = 1;
parameters.CL_delta_e = .1;
parameters.CD_alpha = .006;
parameters.CD_delta_e = .001;
parameters.Cz_beta = .015;
parameters.Cz_delta_r = .001;
parameters.CTz_alpha = .15;
parameters.CTz_delta_e = .1;
parameters.br = .1;
parameters.C_delta_a = .02;
parameters.bp = 1;
parameters.CTy_beta = .02;
parameters.CTy_delta_r = .04;
parameters.bq = .5;
parameters.n_max = 220; % Revs/sec
parameters.D = .254;
parameters.alpha_b = 5;
parameters.g = 9.8; % m/s^2
parameters.rho = 1.204; % kg/m^3


% yes, this is ugly code.
m = 1; % kg
Jxx = .12; % kg/m^2
Jyy = .18; % kg/m^2
Jzz = .15; % kg/m^2
Jxy = .015;
Jxz = 0;
Jyz = 0;
S = 1.5; % m
c = .3 ;% m
A = .25; % m^2
CL_alpha = 1;
CL_delta_e = .1;
CD_alpha = .006;
CD_delta_e = .001;
Cz_beta = .015;
Cz_delta_r = .001;
CTz_alpha = .15;
CTz_delta_e = .1;
br = .1;
C_delta_a = .02;
bp = 1;
CTy_beta = .02;
CTy_delta_r = .04;
bq = .5;
n_max = 220; % Revs/sec
D = .254;
alpha_b = 5;
g = 9.8; % m/s^2
% USING NORTH UP EAST
u = 10; % m/s, forward velocity
v = 0; % m/s, up velocity
w = 0; % m/s, east velocity

% angular velocities
p = 0; % rad/s, roll of airplane (about x)
q = 0; % yaw (about y)
r = 0.01; % pitch (about z)

e0 = cos((15*pi/180)/2);
e1 = 0;
e2 = 0;
e3 = sin((15*pi/180)/2);

% map the body-fixed xyz to inertial frame
pn = 0; % xdotI
pu = 0; % ydotI
pe = 2.5; % zdotI
x = [u;v;w;p;q;r;pn;pu;pe;e0;e1;e2;e3];


figure('WindowState','maximized') %Open the figure and maximize the window
DrawAirplane(xNorth+10, yUp, zEast,e0,e1,e2,e3)
varrho = 0; % rotation angle (rad)
rAxis = [1,0,0]; %axis of rotation
RotationAxisNormed = rAxis/norm(rAxis); %axis of rotation normalized


delta_t = .01;
for ii = 1:1000
    % get control input
    delta_r =  str2double(fileread("left_stick_x"))*.05;
    delta_t =  str2double(fileread("left_stick_y"))*.05;
    delta_e = str2double(fileread("right_stick_y"))*.05;
    delta_a = str2double(fileread("right_stick_x"))*.05;
    if isnan(delta_r)
        delta_r = 0;
    end
    if isnan(delta_t)
        delta_t = 0;
    end
    if isnan(delta_e)
        delta_e = 0;
    end

    if isnan(delta_a)
        delta_a = 0;
    end

    % get the forces together

    % get gravity
    [fxg, fyg, fzg] = gravityForces(parameters.mass, e0, e1, e2, e3);
    % get aero
    % CHECK THESE ARE NOT DIVISION BY ZERO ERRORS
    Va = sqrt(u^2+v^2+w^2);% airspeed
    if Va == 0
        alpha = pi/2;
        beta = pi/2;
    else
        alpha = atan(-v/Va); % the angle of attack in radians
        beta = asin(w/Va);% side-slip angle in radians
    end

    [fxD, fyL, fzbeta, TxAero, Ty, Tz] = AeroForcesAndTorques(x, Va, alpha, beta, delta_e, delta_a, delta_r, parameters);
    % get propeller
    [fxT, TxQ] = PropellerThrustAndTorque(delta_t, u, parameters);
    % sum the forces
    fx = fxg + fxD + fxT;
    fy = fyg + fyL;
    fz = fzg + fzbeta;

    % sum the torques
    Tx = TxAero + TxQ;

    % unnecessary
    %Ty = Ty;
    %Tz = Tz;

    % do some Runge-Kutta
    k1 = AirplaneDynamics(x, fx, fy, fz, Tx, Ty, Tz, parameters.mass, Jxx, Jyy, Jzz, Jxy);
    k2 = AirplaneDynamics(x +delta_t/2*k1 , fx, fy, fz, Tx, Ty, Tz, parameters.mass, Jxx, Jyy, Jzz, Jxy);
    k3 = AirplaneDynamics(x +delta_t/2*k2, fx, fy, fz, Tx, Ty, Tz, parameters.mass, Jxx, Jyy, Jzz, Jxy);
    k4 = AirplaneDynamics(x +delta_t*k3, fx, fy, fz, Tx, Ty, Tz, parameters.mass, Jxx, Jyy, Jzz, Jxy);
    x = x + delta_t/6*(k1 + 2*k2 + 2*k3 + k4);

    %calculate the quaternions
    % varrho = varrho + 0.2; %(rad) rotation angle for quaternions
    % e0 = cos(varrho/2);
    % e1 = sin(varrho/2)*RotationAxisNormed(1);
    % e2 = sin(varrho/2)*RotationAxisNormed(2);
    % e3 = sin(varrho/2)*RotationAxisNormed(3);

    % normalize and get quaternions
    e0 = x(10);%e0 (rad) is a quaternion. It is related to roll, pitch, and yaw
    e1 = x(11);%e1 (rad) is a quaternion. It is related to roll, pitch, and yaw
    e2 = x(12);%e2 (rad) is a quaternion. It is related to roll, pitch, and yaw
    e3 = x(13);%e3 (rad) is a quaternion. It is related to roll, pitch, and yaw
    Den = sqrt(e0^2 + e1^2 + e2^2 + e3^2);
    e0 = e0/Den;
    e1 = e1/Den;
    e2 = e2/Den;
    e3 = e3/Den;
    %Draw the airplane
    clf('reset')
    DrawAirplane(xNorth+ii, yUp, zEast,e0,e1,e2,e3);
    drawnow

    %extract the state variables
    % NOTE: I am aware this is not fast... It's just convenient.
    u = x(1);%u (m/s) is the x-velocity of the airplane in the body frame
    v = x(2);%v (m/s) is the y-velocity of the airplane in the body frame
    w = x(3);%w (m/s) is the z-velocity of the airplane in the body frame
    p = x(4);%p (rad/s) roll rate in the body frame
    q = x(5);%q (rad/s) pitch rate in the body frame
    r = x(6);%r (rad/s) yaw rate in the body frame
    pn = x(7);% pn (m) is the north position of the airplane in the inertial frame
    pu = x(8);%pu (m) is the up position of the airplane in the inertial frame
    pe = x(9);%pe (m) is the east position of the airplane in the inertial frame
end


