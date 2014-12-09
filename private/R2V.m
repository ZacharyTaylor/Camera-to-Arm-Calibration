function [ V ] = R2V( R )
%R2V converts 3x3 rotation matrix into a 1x3 angle-axis vector
%   Inputs -
%   R - a standard 3x3 transformation matrix
%
%   Outputs -
%   V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis 
%   representation of the angle where the unit vector representing the axis
%   has been multipled by the angle of rotation about it

validateattributes(R, {'numeric'},{'size',[3,3]});

R = vrrotmat2vec(double(R));
V = R(1:3)*R(4);
V = V';
end


