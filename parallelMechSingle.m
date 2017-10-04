% SerialLinkage
% 09 / 28 / 2017 Matt Estrada 
% Incorporating gripper analysis ontop of Genliang's dynamic equations for
% the double linear-slider with Astrobee compliance 

clear; close all; clc; 
addpath('adhFunct');         % Functions for adhesive gripper 

%% Run Genliang Simulation 
disp('Genliang Sim')
tic; Dynamic_animation_Sep26; toc

% Rotate into Matt's axes (rotate 180 deg) 
Fout_Gen = [Reac1' Torq']; % Reaction force at distal joint% Rotate
theta = pi; 
rot_Gen2Matt = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];
Fout = (rot_Gen2Matt*Fout_Gen')'; 
t = (1:length(Reac1))*.01; 

%% Hao Calculation of Limit Surface (if running a new design) 
%calculateMomentGivenForceCurved_V2(alpha, phi, F_actual, limit, R, offset)

%% Calculate Adhesive Force 

% Geometry of gripper - go define parameters in that function
gripper = defineGripper();

% Back calculate the specific FBD forces (i.e. req'd adhesive force)
[FBD, success] = calcFBD(gripper,Fout);
% components = [T1 T2 C1 C2] 
% T is tension in adhesive and C is compressive force of outrigger
figure
plot(t,FBD(:,1),t,FBD(:,2),t,gripper.adhLimit * ones(length(t),1))

%% Plotting
% Plot Gripper Constraints in Force Space  
figure
subplot(3,1,1:2)
plot3(Fout(:,1),Fout(:,2),Fout(:,3),'LineWidth',2);
xlabel('F_u [N]')
ylabel('F_v [N]')
zlabel('\tau [Nm]')
set(gca,'fontsize',16);

trans = @(rd) [1 0 0; 0 1 0; rd 0 1];
load('3DscatterLimit_AsymmetricPaper_Sept8')
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];% Get rid of erraneous vals
set(gca,'fontsize',20); hold on;
plotManualIsolines(limit,limit(:,2))

%% Plot Force applied by each adhesive 
subplot(3,1,3)
adhLimitation = ones(length(t),1)*gripper.adhLimit;
plot(t,FBD(:,1),'b',t,FBD(:,2),'r',...
    t,adhLimitation,'k--','LineWidth',2)
xlabel('time [s]')
ylabel('Force [N]')
legend('Adhesive 1 Force', 'Adhesive 2 Force','Adhesive Limit');
set(gca,'fontsize',12); hold on;


