% defineGripper.m
% 9/28/2017 Matt Estrada
% Defining gripper and adhesive geometry for planer, curved adhesive
% gripper with palm 

function gripper = defineGripper()
    trans = @(rd) [1 0 0; 0 1 0; rd 0 1];
    
    r = 9/2*0.0254;              % Distance from object COM to object surface
    d = r + .081; 
    alphad = 11.35;                     % alpha of palm [deg]
    %d_Tintersect = r/cosd(alphad);     % point where tension from adhesive
    alphad=45;                                    %applies no moment
    A = defineGeometry(alphad,r);       % FBD of forces from ICRA 2017 
    A = trans(d)*A; 
    adhLimit = 40;                      % Adhesive limit [N] assuming symmetric
    gripper = struct('alphad',alphad,'r',r,'A',A,'adhLimit',adhLimit); 
end