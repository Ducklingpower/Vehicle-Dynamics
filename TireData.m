clc
clear
close all

%%
% Number of points

    nPoints = 500;
% Pure lateral test case
Fz      = ones(nPoints,1).*500;             % vertical load         (N)
kappa	= ones(nPoints,1).*0;               % longitudinal slip 	(-) (-1 = locked wheel)
alpha	= linspace(deg2rad(-15),deg2rad(15), nPoints)';         % slip angle    	(radians)
gamma	= ones(nPoints,1).*deg2rad(0);               % inclination angle 	(radians)
phit 	= ones(nPoints,1).*0;               % turnslip            	(1/m)
Vx   	= ones(nPoints,1).*16;              % forward velocity   	(m/s)
P       = ones(nPoints,1).*97000;           % pressure              (Pa)

% Create a string with the name of the TIR file
TireFile = 'C:\Users\elija\OneDrive\Documents\MATLAB\Highlander racing\VD\FyP_MF62_SAE.tir';

% Select a Use Mode
useMode = 111;

% Wrap all inputs in one matrix
inputs = [Fz kappa alpha gamma phit Vx P];

% Store the output from mfeval in a 2D Matrix
output = mfeval(TireFile, inputs, useMode);

% Extract variables from output MFeval. For more info type "help mfeval"
Fx = output(:, 1);
Fy = output(:,2);
Mz = output(:,6);
SR = output(:,7);
Mx = output(:,4);
SA = rad2deg(output(:,8)); % Convert to degrees
t = output(:,16);
muy = output(:, 18);



%%
figure
%subplot(2,2,1)
%subplot(2,1,1)
plot(SA, Fy)
grid on
title('Fy-SA')
xlabel('Slip Angle (deg)')
ylabel('Lateral Force (N)')

muy2 = Fy ./ Fz;

figure
plot(SA, muy2)
grid on
title('muy-SA')
xlabel('Slip Angle (deg)')
ylabel('Lateral Friction Coefficient')


