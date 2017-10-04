% Grasp Envel
% 09 / 28 / 2017 Matt Estrada 
% Parsing a .mat file containing a sweep of various incoming conditions 
% Plotting max adhesive force experienced for each trial, as well as the
% grasping envelope 
clear; close all; clc; 
addpath('adhFunct');         % Functions for adhesive gripper 

%% Calculate Adhesive Force 

% Geometry of gripper - go define parameters in that function
gripper = defineGripper();

%% Parsing Genliang's Sweep
load('Reaction_Force.mat')

theta = pi; 
rot_Gen2Matt = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];


[n1 n2 n3] = size(Fu); 
success = zeros(n1,n2); 
FBD = zeros(n1,n2,4); 
tic
for ii = 1:n1
    for jj = 1:n2
        Fout_Gen = [squeeze(Fu(ii,jj,:))...
                    squeeze(Fv(ii,jj,:))...
                    squeeze(Mt(ii,jj,:))];
        Fout = (rot_Gen2Matt*Fout_Gen')'; 
        [thisFBD, thisSuccess] = calcFBD(gripper,Fout);
        success(ii,jj) = thisSuccess; 
        FBD(ii,jj,:) = [max(thisFBD(:,1)) max(thisFBD(:,2)) max(thisFBD(:,3)) max(thisFBD(:,4))] ; 
    end 
end
toc

save('successCompute3','FBD','success'); 
%% Plot Success/Failure (still working on this) 
load('Reaction_Force.mat')
load('successCompute2.mat'); 

[n1 n2 n3] = size(Fu); 

% Genliang told me this was the sweep of velocities he computed
% [U,V] = meshgrid(linspace(-1,1,n1),linspace(0,1,n2));  
% UU = reshape(U,[n1*n2,1]);
% VV = reshape(V,[n1*n2,1]);
% OUTCOME = reshape(success,[n1*n2,1]);
% iFail = find(OUTCOME==0);
% iSucc = find(OUTCOME==1);
% figure
% plot(U(iFail),V(iFail),'rx'); hold on;
% plot(U(iSucc),V(iSucc),'go','MarkerSize',10,'LineWidth',3)
% xlabel('V_u')
% ylabel('V_v') 
% set(gca,'fontsize',12); hold on;

figure
subplot(1,2,1)
mesh(U,V,FBD(:,:,1)'); 
xlabel('v_u')
ylabel('v_v')
zlabel('Max Adhesive Force [N]')
set(gca,'fontsize',16); hold on;
title('Adhesive 1')

subplot(1,2,2)
mesh(U,V,FBD(:,:,2)'); 
xlabel('v_u')
ylabel('v_v')
zlabel('Max Adhesive Force [N]')
set(gca,'fontsize',16); hold on;
title('Adhesive 2') 


success50 = successEnvel(FBD,50); 
success100 = successEnvel(FBD,100); 
success150 = successEnvel(FBD,150); 


figure
spy(success150,'c');hold on;
spy(success100,'g');
spy(success50); 
legend('150','100','50'); 
title('Quick Visualization of Grasping Envelope') 


