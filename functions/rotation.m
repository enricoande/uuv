function x = rotation(euler_angles,X)
% rotation.m     e.anderlini@ucl.ac.uk     18/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function applies a rotation across all Euler angles to a given
% vector.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simplify notation:
cphi = cos(euler_angles(1));
sphi = sin(euler_angles(1));
cthe = cos(euler_angles(2));
sthe = sin(euler_angles(2));
cpsi = cos(euler_angles(3));
spsi = sin(euler_angles(3));

%% Find the rotation matrix:
R = [cthe*cpsi,sphi*sthe*cpsi-cphi*spsi,cphi*sthe*cpsi+sphi*spsi;...
    cthe*spsi,sphi*sthe*spsi+cphi*cpsi,cphi*sthe*spsi-sphi*cpsi;...
    -sthe,cthe*sphi,cthe*cphi];

%% Find the rotated vector:
x = R*X;

end