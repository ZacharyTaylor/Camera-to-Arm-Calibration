function [ V ] = T2V( T )
%T2V converts 4x4 transformation matrix into a 1x6 vector
%   Inputs -
%   T - a standard 4x4 transformation matrix
%
%   Outputs -
%   V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
%   and rx,ry,rz is an angle-axis representation of the angle where the
%   unit vector representing the axis has been multipled by the angle of
%   rotation about it

validateattributes(T, {'numeric'},{'size',[4,4]});

T = double(T);

V(1:3) = T(1:3,4);
V(4:6) = R2V(T(1:3,1:3));

end

