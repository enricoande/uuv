function S = skew(x)
% skew.m      e.anderlini@ucl.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function takes a 3D vector and returns a 3D skew symmetric matrix.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Check if size is OK:
if length(x)>3
    error('Vector should have maximum length of 3');
end

% Build skew symmetric matrix:
S = [0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];

end