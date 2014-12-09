function [ T ] = V2T( V )
%V2T converts 1x6 vector into 4x4 transformation matrix
%   Inputs -
%   V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
%   and rx,ry,rz is an angle-axis representation of the angle where the
%   unit vector representing the axis has been multipled by the angle of
%   rotation about it
%
%   Outputs -
%   T - a standard 4x4 transformation matrix

validateattributes(V, {'numeric'},{'size',[1,6]});

V = double(V);

T = eye(4);
T(1:3,4) = V(1:3);
T(1:3,1:3) = V2R(V(4:6));


end

