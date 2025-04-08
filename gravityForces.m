function [fxg,fyg,fzg] = gravityForces(m,e0,e1,e2,e3)
% [fxg,fyg,fzg] = gravityForces(m,e0,e1,e2,e3)
%
%OUTPUTs:
% fxg: (N) gravity force in the body-fixed x-direction
% fyg: (N) gravity force in the body-fixed y-direction
% fzg: (N) gravity force in the body-fixed z-direction
%
%INPUTS:
% m: (kg) airplane mass
% e0: (unitless) quaternion
% e1: (unitless) quaternion
% e2: (unitless) quaternion
% e3: (unitless) quaternion
g = 9.8; % m/s^2
fxg = -m*g*2*(e0*e3+e1*e2);
fyg = -m*g*(e0^2-e1^2+e2^2-e3^2);
fzg = -m*g*2*(e2*e3-e0*e1);
