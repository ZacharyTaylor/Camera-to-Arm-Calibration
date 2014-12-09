function [ tform, tformStd ] = ConvertTformSystem( aaVec, aaStd )
%CONVERTTFORMSYSTEM Converts from angle axis form to tform matrix
%   Inputs
%   aaVec- angle axis vector
%   aaStd- standard deviation of angle axis vector
%
%   Outputs
%   tform- standard tform matrix
%   tformStd- standard deviation of tform matrix

validateattributes(aaVec, {'numeric'},{'size',[1,6]});
validateattributes(aaStd, {'numeric'},{'size',[1,6]});

aaVec = double(aaVec);
aaStd = double(aaStd);

%Transformation matrix form
tform = V2T(aaVec);
tformStd = zeros(4,4,1000);
for idx = 1:size(tformStd,3)
    tformStd(:,:,idx) = V2T(aaVec + aaStd.*randn(1,6));
end
tformStd = std(tformStd,0,3);

end

