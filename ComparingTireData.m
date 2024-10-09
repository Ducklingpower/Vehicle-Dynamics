clc
clear
close all





%% Reading tire data

Lat_Tire  = load('C:\Users\elija\OneDrive\Documents\MATLAB\Highlander racing\VD\lateral_tire_test.mat');
Lat_Fx = Lat_Tire.FX;
Lat_Fy = Lat_Tire.FY;
Lat_Mz = Lat_Tire.MZ;
Lat_Sr = Lat_Tire.SR;
Lat_Mx = Lat_Tire.MX;
Lat_Sa = Lat_Tire.SA;

Lat_Tire  = load('C:\Users\elija\OneDrive\Documents\MATLAB\Highlander racing\VD\longitudinal_tire_test.mat');
Long_Fx = Lat_Tire.FX;
Long_Fy = Lat_Tire.FY;
Long_Mz = Lat_Tire.MZ;
Long_Sr = Lat_Tire.SR;
Long_Mx = Lat_Tire.MX;
Long_Sa = Lat_Tire.SA;


%%
figure(1)
plot(Lat_Sa,Lat_Fy,LineWidth = 1,LineStyle=":")
hold on 
ForceZ = [220 440 660 880 1100];
for i = 1:5
% Number of points

    nPoints = 500;
% Pure lateral test case
Fz      = ones(nPoints,1).*ForceZ(i);             % vertical load         (N)
kappa	= ones(nPoints,1).*0;               % longitudinal slip 	(-) (-1 = locked wheel)
alpha	= linspace(deg2rad(-15),deg2rad(15), nPoints)';         % slip angle    	(radians)
gamma	= ones(nPoints,1).*deg2rad(0);               % inclination angle 	(radians)
phit 	= ones(nPoints,1).*0;               % turnslip            	(1/m)
Vx   	= ones(nPoints,1).*16;              % forward velocity   	(m/s)
P       = ones(nPoints,1).*97000;           % pressure              (Pa)

% Create a string with the name of the TIR file
TireFile = 'C:\Users\elija\OneDrive\Documents\MATLAB\Highlander racing\VD\TireDataFile.tir';

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



plot(-SA,Fy,"o")
xlabel("SA")
ylabel("Fy")
end

legend("Lat Tire Data","Fz = 220", "Fz = 440", "Fz = 660","Fz = 880","Fz = 1100")






%% with other file 

%%
figure(2)
plot(Lat_Sa,Lat_Fy,LineWidth = 1,LineStyle=":")
hold on 
ForceZ = [220 440 660 880 1100];
for i = 1:5
% Number of points

    nPoints = 500;
% Pure lateral test case
Fz      = ones(nPoints,1).*ForceZ(i);             % vertical load         (N)
kappa	= ones(nPoints,1).*0;               % longitudinal slip 	(-) (-1 = locked wheel)
alpha	= linspace(deg2rad(-15),deg2rad(15), nPoints)';         % slip angle    	(radians)
gamma	= ones(nPoints,1).*deg2rad(0);               % inclination angle 	(radians)
phit 	= ones(nPoints,1).*0;               % turnslip            	(1/m)
Vx   	= ones(nPoints,1).*16;              % forward velocity   	(m/s)
P       = ones(nPoints,1).*97000;           % pressure              (Pa)

% Create a string with the name of the TIR file
TireFile = 'C:\Users\elija\OneDrive\Documents\MATLAB\Highlander racing\VD\TireDataFileScaled.tir';

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



plot(-SA,Fy,"o")
xlabel("SA")
ylabel("Fy")
end

legend("Lat Tire Data Scaled","Fz = 220", "Fz = 440", "Fz = 660","Fz = 880","Fz = 1100")




