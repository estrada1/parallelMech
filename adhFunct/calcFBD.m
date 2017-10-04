% calcFBD.m
% 09 / 28 / 2017 Matt Estrada
% Taking the results of a simulation and backing out 
% Input     Fout dimension (nx3) (results from simulation)
%           gripper (object containing gripper parameters)
% Output    FBD [T1 T2 C1 C2] as defined in EstradaICRA2017
%           success whether or not the simulation exceeded adhesion's
%           capabilities, based on gripper.adhLimit. You can also calculate
%           this separately from FBD
function [FBD, success] = calcFBD(gripper,Fout)
    FBD = zeros(length(Fout),4); 
    for nn = 1:length(Fout)
        thisForce = Fout(nn,:);
        FBD(nn,:) = ( lsqnonneg(gripper.A,thisForce') )';
    end
    
    adh1Fail = any(FBD(:,1) > gripper.adhLimit); 
    adh2Fail = any(FBD(:,2) > gripper.adhLimit); 
    success = ~(adh1Fail || adh2Fail); 
end