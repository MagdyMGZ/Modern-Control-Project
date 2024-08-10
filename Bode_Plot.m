close all;
clear;
clc;
%% Bode Plot
%% [1] G(s) & H(s) Definition using tf command
num_G = [0 0 1];
denum_G = [1 1 0];
num_H = [0 0 1];
denum_H = [0 0 1];
G = tf(num_G,denum_G);
H = tf(num_H,denum_H);
%% [2] Plot G(s) Response for a Unit Step Input
figure;
step(G);
title('Step Response of G(s)');
grid on;
% Unstable Response + Expected Behavior (poles = 0 , -1)
poles_G = eig(G); % = pole(G);
if (real(poles_G(:)) < 0)
    disp('G(s) Output Due to Unit Step input is Stable');
elseif (real(poles_G(:)) == 0)
    disp('G(s) Output Due to Unit Step input is Critically Stable');
else
    disp('G(s) Output Due to Unit Step input is Unstable');
end
%% [3] Closed Loop Feedback System Definition
CL_F = feedback(G,H,-1); % -1 = -ve feedback
CL_Formula = G/(1+G*H); % Closed Loop Formula with unsimplified form
CL_Formula_Simplified = minreal(CL_Formula); % Closed Loop Formula with simplified form
% Comparison between Closed loop system in feedback command and with G/(1+GH) formula using bode plot
figure;
bode(CL_F,'r');
hold on;
bode(CL_Formula,'b--o');
hold off;
legend('CL - Feedback',' CL - Formula');
title('Bode Plot Comparison for Closed Loop Transfer Function');
grid on;
%% [4] Plot Output of Closed Loop
figure;
step(CL_F);
title('Step Response of Closed Loop Transfer Function');
grid on;
poles_CL = eig(CL_F); % = pole(G)
if (real(poles_CL(:)) < 0)
    disp('Closed Loop System Output Due to Unit Step input is Stable');
elseif (real(poles_CL(:)) == 0)
    disp('Closed Loop System Output Due to Unit Step input is Critically Stable');
else
    disp('Closed Loop System Output Due to Unit Step input is Unstable');
end
%% [5] Poles Locations of the Closed Loop Transfer Function
figure;
pzmap(CL_F);
title('Poles And Zeros for Closed Loop Transfer Function');
grid on;
%% [6] Closed Loop Step Response Characteristics
% Peak Amplitude = 1.16, Overshoot = 16.3 % @ Time = 3.59 Sec
% Settling Time = 8.08 Sec
% Steady State Final Value = 1 
%% [7] Steady State Error to a Unit Step Input
% ess = 0 + System type 1
%% [8] Ramp Input Response 
integrator = tf([0 1],[1 0]);
figure;
step(CL_F.*integrator,'r');
hold on;
step(integrator,'b--o');
hold off;
legend('CL TF Ramp Response',' Ramp Input Response');
title('Ramp Response Comparison for Input and Closed Loop Transfer Function');
grid on;
% ess = 1
%% [9] Plot Frequency Response of the System
% ωpc, ωgc, PM, GM are got from Open loop Gain G(s).H(s)
figure;
margin(G*H);
grid on;
allmargin(G*H)