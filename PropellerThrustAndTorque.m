function [fxT, TxQ] = PropellerThrustAndTorque(delta_t,ur,consts)
%[fxT, TxQ] = PropellerThrustAndTorque(delta_t,ur,consts)
%
%OUTPUTS:
%fxT: (N) propeller thrust in the body-fixed x-axis
%TxQ: (N m) propeller torque around the body-fixed x-axis
%
%INPUTS:
%delta_t: (0-1) throttle command
%ur: (m/s) relative airspeed in the body-fixed x-direction
%consts: A structure with at least the following items
%  consts.n_max: (rev/s) Maximum propeller speed
%  consts.D: (m) propeller diameter
%  consts.alpha_b: (inch) propeller pitch
%  consts.rho: (kg/m3) air density
  if delta_t == 0
    TxQ =0;
    fxT = 0;
    return
  end
  n = consts.n_max*(1-(1-delta_t)^2);
  J = ur/(n*consts.D);
  zQ = 1/(1+exp(10*(.16+.05*consts.alpha_b-J)));
  CQL = .0121-.017*(J-.1*(consts.alpha_b-5));
  CQu = .0057+.00125*(consts.alpha_b-5)-.0005*J;
  CQ = zQ*CQL + (1-zQ)*CQu;

  TxQ = consts.rho*n^2*consts.D^5*CQ;

  z = 1/(1+exp(20*(.14+.018*consts.alpha_b-.6*J)));
  CTL = .11-.18*(J-.105*(consts.alpha_b-5));
  CTu = .1 + .009*(consts.alpha_b-5)-.065*J;
  CT = z*CTL + (1-z)*CTu;
  fxT = consts.rho*n^2*consts.D^4*CT;
  
