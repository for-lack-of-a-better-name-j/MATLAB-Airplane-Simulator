%This file acts like a wind tunnel to test the function
%AeroForcesAndTorques:
%
% [fxD,fyL,fzbeta,TxAero,Ty,Tz] = ...
%    AeroForcesAndTorques(x,Va,alpha,beta,...
%    delta_e,delta_a,delta_r,consts)


close all
clear all
clc

%% set the airplane constants
%set the airplane constants
consts.rho = 1.27; %(kg/m3) air density
%Aero constants
consts.A = 0.25;
consts.S = 1.5;
consts.c = 0.3;
consts.CL_alpha = 1;
consts.CL_delta_e = 0.1;
consts.CD_alpha = 0.006;
consts.CD_delta_e = 0.001;
consts.Cz_beta = 0.015;
consts.Cz_delta_r = 0.001;
consts.CTz_alpha = 0.15;
consts.CTz_delta_e = 0.1;
consts.br = 0.1;
consts.C_delta_a = 0.02;
consts.bp = 1;
consts.CTy_beta = 0.02;
consts.CTy_delta_r = 0.04;
consts.bq = 0.5;


%% test the effects of the elevator command delta_e
n = 100;
x = zeros(13,1); x(10) = 1;
beta = 0;
alpha = 0; %(rad) angle of attack
delta_a = 0;
delta_r = 0;
delta_e = [-1, -0.5, 0, 0.5, 1];
Va = [...
    linspace(0,10,n);...
    linspace(0,10,n);...
    linspace(0,10,n);...
    linspace(0,10,n);...
    linspace(0,10,n)]';
Tz = zeros(size(Va));
for ii = 1:length(Va(:,1))
    for jj = 1:length(Va(1,:))
        [~,~,~,~,~,Tz(ii,jj)] = ...
            AeroForcesAndTorques(x,Va(ii,jj),alpha,beta,...
            delta_e(jj),delta_a,delta_r,consts);
    end
end

figure, plot(Va, Tz)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Pitch Torque (N m)')
title('Effect of Elevator Angle')
legend('\delta_e=-1','\delta_e=-0.5','\delta_e=0',...
    '\delta_e=0.5','\delta_e=1','Location','NorthWest')

%% test the effect of the angle of attack (alpha)

delta_e = 0*pi/180; %(rad) elevator angle
alpha = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';
fxD = zeros(size(Va));
Tz = zeros(size(Va));
fz = zeros(size(Va));
for ii = 1:length(Va(:,1))
    for jj = 1:length(Va(1,:))
        [fxD(ii,jj),fyL(ii,jj),fzbeta,TxAero,Ty,Tz(ii,jj)] = ...
            AeroForcesAndTorques(x,Va(ii,jj),alpha(ii,jj),beta,...
            delta_e,delta_a,delta_r,consts);
    end
end

figure, plot(Va, fxD)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Drag (N)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20','Location','SouthWest')

figure, plot(Va, fyL)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Lift (N)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20','Location','NorthWest')

figure, plot(Va, Tz)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Pitch Torque (N m)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20','Location','NorthWest')



%% test the effects of the rudder (delta_r)

beta = 0; %(rad) sideslip angle
p=0;
r=0;
delta_a = 0;
delta_r = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';
fzbeta = zeros(size(Va));
Ty = zeros(size(Va));
for ii = 1:length(Va(:,1))
    for jj = 1:length(Va(1,:))
        [fxD,fyL,fzbeta(ii,jj),TxAero,Ty(ii,jj),Tz] = ...
            AeroForcesAndTorques(x,Va(ii,jj),alpha,beta,...
            delta_e,delta_a,delta_r(ii,jj),consts);
    end
end

figure, plot(Va, fzbeta)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Lateral Force (N)')
title('Effect of the Rudder Angle')
legend('\delta_r=-20','\delta_r=-10','\delta_r=0',...
    '\delta_r=10','\delta_r=20','Location','NorthWest')

figure, plot(Va, Ty)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Yaw torque (N m)')
title('Effect of the Rudder Angle')
legend('\delta_r=-20','\delta_r=-10','\delta_r=0',...
    '\delta_r=10','\delta_r=20','Location','NorthWest')

%% test the effects of the aileron (delta_a)

beta = 0; %(rad) side slip angle
p=0;
r=0;
delta_r = 0;
delta_a = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';
TxAero = zeros(size(Va));
for ii = 1:length(Va(:,1))
    for jj = 1:length(Va(1,:))
        [fxD,fyL,fzbeta,TxAero(ii,jj),Ty,Tz] = ...
            AeroForcesAndTorques(x,Va(ii,jj),alpha,beta,...
            delta_e,delta_a(ii,jj),delta_r,consts);
    end
end


figure, plot(Va, TxAero)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Roll torque (N m)')
title('Effect of the Aileron Angle')
legend('\delta_a=-20','\delta_a=-10','\delta_a=0',...
    '\delta_a=10','\delta_a=20','Location','NorthWest')

x = (0.1:0.1:1.3)';
Va = 6;
alpha = 0.25;
beta = -0.34;
delta_e = 0.2;
delta_a = -0.3;
delta_r = -0.5;
[fxD,fyL,fzbeta,TxAero,Ty,Tz] = ...
    AeroForcesAndTorques(x,Va,alpha,beta,...
    delta_e,delta_a,delta_r,consts);
disp(['You got:       ',num2str([fxD,fyL,fzbeta,TxAero,Ty,Tz])])
disp('You should get: -0.034367      1.5431    0.032004    -0.45144    -0.20405    -0.15858')