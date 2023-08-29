clear; close all; clc;
%% Problem 1c
Q = 5; w0 = 10;
F = RR_tf([w0/Q 0],[1 (1/Q)*w0 w0^2]);
figure(1)
RR_bode(F)
%% Problem 1d
syms R C L c1 s Vi
A = [0,1,s*L,0,0,0;
    0,0,1,-1,0,0;
    -s*C,s*C,0,-1,0,0;
    0,0,0,1,-1,-1;
    1,0,0,0,-R,0;
    1,0,0,0,0,-R/c1];
b = [Vi;0;0;0;0;0];
x = A\b;
Vi_Input = simplify(x(1))
