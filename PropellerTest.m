close all
clear all
clc

%propeller constants
consts.rho = 1.2682; %(kg/m3) air density
consts.D = 10*0.0254; %(m) propeller diameter
consts.alpha_b = 5; %(inch) propeller pitch
consts.n_max = 220; %(rev/s) maximum propeller speed

%Vector of relative airspeeds
ur = logspace(0,1,20);
M = length(ur);

%Vector of throttle commands
delta_t = 0.14:0.01:1;
N =  length(delta_t);

%Allocate memory to store the torque coefficient CQ, thrust coefficient CT,
% and advance ratio J
CQ = zeros(1,N*M);
CT = zeros(1,N*M);
J = zeros(1,N*M);
aa = 1;
for ii = 1:N
    for jj = 1:M
        %Propeller speed (rev/s)
        n = consts.n_max * (1 - (1 - delta_t(ii))^2);
        %Advance ratio (unitless)
        J(aa) = ur(jj)/n/consts.D;
        %Calculate the propeller thrust (N) and torque (N m)
        [fxT, TxQ] = PropellerThrustAndTorque(delta_t(ii),ur(jj),consts);
        %Determine the thrust and torque coefficients (unitless)
        CT(aa) = fxT/(consts.rho*n^2*consts.D^4);
        CQ(aa) = TxQ/(consts.rho*n^2*consts.D^5);
        aa = aa+1;
    end
end
    
%Plot the thrust and torque coefficients
figure,
plot(J,CT,'.')
grid on
title('Thrust Coefficient')
ylabel('C_T')
xlabel('J = u_r/n/D')

figure, 
plot(J,CQ,'.')
grid on
title('Torque Coefficient')
ylabel('C_Q')
xlabel('J = u_r/n/D')

