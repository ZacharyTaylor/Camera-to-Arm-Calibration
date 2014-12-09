function [ R ] = V2R( V )
%V2R converts a 1x3 angle-axis vector into a 3x3 rotation matrix 
%   Inputs -
%   V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis 
%   representation of the angle where the unit vector representing the axis
%   has been multipled by the angle of rotation about it
%
%   Outputs -
%   R - a standard 3x3 transformation matrix

validateattributes(V, {'numeric'},{'size',[1,3]});

V = double(V(:));

s = norm(V);
if(s == 0)
    R = eye(3);
    return;
end

V = [V/s; s];
R = vrrotvec2mat(V);

end

