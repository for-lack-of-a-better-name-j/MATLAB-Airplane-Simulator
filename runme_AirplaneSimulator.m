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
m = 1 % kg
Jxx = .12 % kg/m^2
Jyy = .18 % kg/m^2
Jzz = .15 % kg/m^2
Jxy = .015
Jxz = 0
Jyz = 0
S = 1.5 % m
c = .3 % m
A = .25 % m^2
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
g = 9.8 % m/s^2

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
for ii = 1:100
    %calculate the quaternions


    
    varrho = varrho + 0.2; %(rad) rotation angle for quaternions
    e0 = cos(varrho/2);
    e1 = sin(varrho/2)*RotationAxisNormed(1);
    e2 = sin(varrho/2)*RotationAxisNormed(2);
    e3 = sin(varrho/2)*RotationAxisNormed(3);
    %Draw the airplane
    clf('reset')
    DrawAirplane(xNorth+ii, yUp, zEast,e0,e1,e2,e3);
    drawnow
end


