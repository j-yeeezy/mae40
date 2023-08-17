% script RR_Ex10_02_passive_filters
% Solves the basic equations of four simple passive filters.
% These examples are easy to solve by hand, but illustrate how
% to put a few linear equations into A*x=b form and solve using Matlab.
% Renaissance Robotics codebase, Chapter 10, https://github.com/tbewley/RR
% Copyright 2023 by Thomas Bewley, distributed under Modified BSD License.

clear, close all, syms s Vi Vs C L R R1 R2 Vd Rd Cd

%% Problem 1
% Second-order low-pass LC filter: Solve for Vo as a function of Vi
% x={I_L,Ic,Ir,Vo}  <-- unknown vector
A  =[ 1  -1   -1    0;    % I_L - Ic - Ir = 0
     L*s  0   0     1;    % L*s*I_L + Vo = Vi
      0  -1   0    C*s;   % C*s*Vo - Ic = 0
      0   0   1   -1/R];  % Ir - Vo/R = 0
b  =[ 0; Vi; 0; 0];
x=A\b; Vo_LPF2_undamped=simplify(x(4))

%% Problem 2 
omega4=10; zeta = 0.1;
F_LPF2_undamped=RR_tf([omega4^2],[1 2*omega4*zeta omega4^2]);
figure(3), RR_bode(F_LPF2_undamped)
hold on
zeta = 0.7;
F_LPF2_damped=RR_tf([omega4^2],[1 2*omega4*zeta omega4^2]);
RR_bode(F_LPF2_damped)
zeta = 1;
F_LPF2_damped=RR_tf([omega4^2],[1 2*omega4*zeta omega4^2]);
RR_bode(F_LPF2_damped)

% Second-order low-pass LC filter: Solve for Vo as a function of Vi
% x={I_L,Ic,Ir,Vo}  <-- unknown vector
A  = [1,-1,-1,0,0,0;
      0,1,0,0,-s*C,0;
      s*L,0,0,0,1,0;
      0,0,1,-1,0,0;
      0,0,0,1,0,-s*Cd;
      0,0,-Rd,0,1,-1];
b  =[0;0;Vi;0;0;0];
x=A\b; Vo_LPF2_damped=simplify(x(5))
omega4=10; F_LPF2_damped=RR_tf([omega4^2],[1 0 omega4^2]);
figure(4), RR_bode(F_LPF2_damped)
