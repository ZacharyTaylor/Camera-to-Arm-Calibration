function [ err ] = ProjectError( points, cameraParams, worldPoints, armPose, inliers, est )
%PROJERROR Projects checkerboards into camera frame and finds error betwen
%projected and actual position.
%   Inputs-
%   points- nx2xm matrix of the position of checkerboard corner n in image
%       m. given as [x,y]
%   cameraParams- camera parameters object giving camera intrinsics
%   worldPoints- nx2 matrix giving the position of the checkerboard corners
%       before being transformed
%   armPose- 4x4xm matrix, gives transformation from robot arms base to end
%       effector for each of the m images
%   inliers- percent of data to take as inliers (from 0 to 100)
%   est- estimated parameters values given as 1x13 vector
%       [baseTformVector,endTformVector,squareSize].
%
%   Outputs -
%   err- mean projection error of inliers in pixels

%extract transforms from estimate
baseT = V2T(est(1:6));
gripT = V2T(est(7:12));
squareSize = est(13);

%add square size to chessboard
worldPoints = [squareSize.*worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)]';

%get camera matrix
K = cameraParams.IntrinsicMatrix';

%get distortion parameters
p = cameraParams.TangentialDistortion;
if(length(p) < 2)
    p(2) = 0;
end
k = cameraParams.RadialDistortion;
if(length(k) < 3)
    k(3) = 0;
end

err = nan(size(points,1),size(points,3));
%loop over each armPose
for i = 1:size(armPose,3)
    %transform chessboard points from the world into the image
    projected = ([1,0,0,0;0,1,0,0;0,0,1,0]*baseT*armPose(:,:,i)*gripT*worldPoints)';

    x = projected(:,1)./projected(:,3);
    y = projected(:,2)./projected(:,3);
    r2 = x.^2 + y.^2;

    %find tangental distortion
    xTD = 2*p(1)*x.*y + p(2).*(r2 + 2*x.^2);
    yTD = p(1)*(r2 + 2*y.^2) + 2*p(2)*x.*y;

    %find radial distortion
    xRD = x.*(1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3); 
    yRD = y.*(1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3); 

    %recombine and include camera intrinsics
    projected = [xRD + xTD, yRD + yTD, ones(size(x,1),1)];
    projected = K*projected';
    projected = projected(1:2,:)';

    %find error in projection
    err(:,i) = sum((projected - points(:,:,i)).^2,2);
end

%remove invalid points
err = err(~isnan(err(:)));

%drop the largest values as outliers and take mean
err = sort(err(:));
err = mean(err(1:floor((inliers/100)*size(err,1))));

%return mean error in pixels
err = sqrt(err);