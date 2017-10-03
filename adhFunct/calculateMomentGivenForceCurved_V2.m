function [M1, M2] = calculateMomentGivenForceCurved_V2(alpha, phi, F_actual, limit, R, offset)

% 02/28/2017, Hao Jiang
% Theory: Check for all the six cases given any Fx and Fy, there are always at most 2 cases valid, each for one direction of the moment limit
% IMPORTANT TODO: double check the signs for M1 and M2 based on Cartesian coordinates
% Function: calculateMomentGivenForceCurved_V2
% Input: alpha (defined in the paper)
%        phi (the angle between the external force and x axis)
%        F_actual (external force)
%        limit (adhesion limit of each side of the adhesive film, assuming both sides equal)
%        R (radius of the cylindrical object)
%        offset (the loading point offset from the intersection of the tangent lines in the y direction, assuming x direction aligned with the gripper, i.e., the loading point is at the center)
% Output: M1 (moment limit in one direction)
%         M2 (moment limit in the other direction)

caseSet = [];
MomentSet = [];
ForceSet = [];
% case 1: T2 and two compressions
if 1
    T1 = 0;
    T2 = limit;
    N1 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
    N2 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
    if N1 >= -0.02 && N2 >= -0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 1];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 2: T1 and two compressions
if 1
    T1 = limit;
    T2 = 0;
    N1 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
    N2 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
    if N1 >= -0.02 && N2 >= -0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 2];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 3: two tensions and N2, T1 is limit
if 1
    T1 = limit;
    N1 = 0;
    N2 = F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha);
    T2 = F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T1 * sin(alpha) * sin(alpha);
    if N2 >= -0.02 && T2 >= -0.02 && T2 <= limit+0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 3];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 4: two tensions and N2, T2 is limit
if 1
    T2 = limit;
    N1 = 0;
    N2 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-cos(2*alpha));
    T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T2 * sin(alpha) * sin(alpha)) / (-cos(2*alpha));
    if N2 >= -0.02 && T1 >= -0.02 && T1 <= limit+0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 4];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 5: two tensions and N1, T1 is limit
if 1
    T1 = limit;
    N2 = 0;
    N1 = (F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha)) / cos(2*alpha);
    T2 = (F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T1 * sin(alpha) * sin(alpha)) / cos(2*alpha);
    if N1 >= -0.02 && T2 >= -0.02 && T2 <= limit+0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 5];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 6: two tensions and N1, T2 is limit
if 1
    T2 = limit;
    N2 = 0;
    N1 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-1);
    T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T2 * sin(alpha) * sin(alpha)) / (-1);
    if N1 >= -0.02 && T1 >= -0.02 && T1 <= limit+0.02
        Moment = (N2 - N1) * R * tan(alpha);
        caseSet = [caseSet, 6];
        MomentSet = [MomentSet, Moment];
        ForceSet = [ForceSet; T1, T2, N1, N2];
    end
end
% case 7: default
if isempty(caseSet)
    Moment = NaN;
    MomentSet = [MomentSet, Moment];
end

M1 = max(MomentSet);
M2 = min(MomentSet);
M1 = M1 - F_actual * cos(phi) * offset; % loading point offset from the intersection of the tangent lines
M2 = M2 - F_actual * cos(phi) * offset;

%  % Display and check the cases
% caseSet
% MomentSet
% ForceSet