close all;
clear;
clc;
%% State Space Representation
%% [1] Finding Transfer Function Hand Analysis
% T.F. = C ((SI-A)^-1) B + D
% T.F. = 1 / (s^2 + 5s + 6)
%% [2] Finding Transfer Function From States and Output Matrices
syms s; % Complex Frequency Domain
A = [0 1 ; -6 -5]; % A = System (State) Matrix
B = [0 ; 1];       % B = Input Matrix
C = [1 0];         % C = Output Matrix
D = 0;             % D = Feedforward Matrix
system = ss(A,B,C,D);
TF_1 = C * (inv(s .* eye(2) - A)) * B + D; % Controllable Form
[num,denum] = ss2tf(A,B,C,D); % ss2tf Command Convert A,B,C,D to TF
TF_2 = tf(num,denum);
%% [3] State Transition Matrix
syms t; % Time Domain
PHI_S = inv(s .* eye(2) - A);
PHI_T = ilaplace(PHI_S);
PHI_0 = subs(PHI_T,t,0); % subs(function,old,new) PHI_0 = I
if (unique(PHI_0 == eye(2)))
    disp('∅(0) = I');
else
    disp('∅(0) ≠ I');
end
%% [4] Derivative of State Transition Matrix
PHI_Diff = diff(PHI_T,t);
APHI = A * PHI_T;
if (unique(PHI_Diff == APHI))
    disp('d∅/dt = A.∅(t)');
else
    disp('d∅/dt ≠ A.∅(t)');
end
%% [5] Controllability and Observability of this SSR
% Checking Contrillability Δ(Qc) ≠ 0 (Expected Controllable)
Rank_C = rank(ctrb(ss(system)));
degree = length(denum) - 1;
if (Rank_C == degree)
    disp(['System is Controllable with Rank = ',num2str(degree)]);
else
    disp('System is Uncontrollable');
end
% Checking Observability Δ(Qo) ≠ 0 (Expected Observable)
Rank_O = rank(obsv(ss(system)));
if (Rank_O == degree)
    disp(['System is Observable with Rank = ',num2str(degree)]);
else
    disp('System is Non Observable');
end
%% [6] Unforced (Homogeneous) Solution of the States x(t) and The Unforced Response y(t)
x_0 = [0 ; 1];
x_unforced = PHI_T * x_0;
y_unforced = C * x_unforced;
%% [7] The Unhomogeneous System
U = 1/s;
x_forced = x_unforced + ilaplace((inv(s.*eye(2) - A)) * B * U);
y_forced = C * x_forced;