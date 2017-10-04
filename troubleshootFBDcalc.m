close all; clear; clc; 
addpath('adhFunct');         % Functions for adhesive gripper 


tic; Dynamic_animation_Sep26; toc
Fout_Gen = [Reac1' Torq']; % Reaction force at distal joint% Rotate
theta = pi; 
rot_Gen2Matt = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];
Fout = (rot_Gen2Matt*Fout_Gen')'; 
t = (1:length(Reac1))*.01; 


% load('forceProfile')
% Geometry of gripper - go define parameters in that function
gripper = defineGripper();

% % Rotate
% theta= pi; 
% rot_Gen2Matt = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];
% Fout = (rot_Gen2Matt*Fout')';

% Back calculate the specific FBD forces (i.e. req'd adhesive force)
[FBD, success] = calcFBD(gripper,Fout);
Fout_verify = (gripper.A*FBD')'; 


verifyPlots(Fout,Fout_verify,FBD,gripper,t,'Control')

%verifyPlots(Fout_rot,Fout_verify,FBD,gripper,t,'Control')

%%

% theta = pi; F
% rot_Gen2Matt = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];
% rout

function verifyPlots(Fout,Fout_verify,FBD,gripper,t,trial)
    %% Plot the FBD components 
    
    figure
    plot(t,Fout(:,1),t,Fout(:,2),t,Fout(:,3),'LineWidth',2); hold on; 
    plot(t,Fout_verify(:,1),'--',t,Fout_verify(:,2),'--',t,Fout_verify(:,3),'--','LineWidth',2); hold on; 
    legend('Fx', 'Fy','Mz')
    xlabel('Time [s]')
    ylabel('Force [N]')
    title(trial)
    
    figure
    subplot(2,1,1)
    plot(t,FBD(:,1),t,FBD(:,2),t,gripper.adhLimit * ones(length(t),1)); 
    legend('Adhesive 1', 'Adhesive 2')
    xlabel('Time [s]')
    ylabel('Force [N]')
    subplot(2,1,2)
    plot(t,FBD(:,3),t,FBD(:,4))
    legend('Compression 1', 'Compression 2')
    xlabel('Time [s]')
    ylabel('Force [N]')
    title(trial)

    
end