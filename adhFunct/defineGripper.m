% defineGripper.m
% 9/28/2017 Matt Estrada
% Defining gripper and adhesive geometry for planer, curved adhesive
% gripper with palm 

function gripper = defineGripper()
    r = 9/2*0.0254 + .081;              % Distance from object COM to object surface
    alphad = 11.35;                     % alpha of palm [deg]
    A = defineGeometry(alphad,r);       % FBD of forces from ICRA 2017 
    adhLimit = 40;                      % Adhesive limit [N] assuming symmetric
    gripper = struct('alphad',alphad,'r',r,'A',A,'adhLimit',adhLimit); 
end