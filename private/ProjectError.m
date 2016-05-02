function [ err, proj, projEst ] = ProjectError( points, cameraParams, worldPoints, armPose, inliers, est )
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
%   proj- nx2xm matrix of the position of checkerboard corner n in image
%       m. given as [x,y] as guessed by the est
%   pojEst- 12x2xm matrix of the position of [base origin, basex, basey, 
%           basez,tcp origin, tcp x, tcp y, tcp z, grid x, grid y, grid z] in image
%       m. given as [x,y] as guessed by the est

%extract transforms from estimate
baseT = V2T(est(1:6));
gripT = V2T(est(7:12));
squareSize = est(13);

%add square size to chessboard
worldPoints = [squareSize.*worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)]';

%get camera matrix
K = cameraParams.IntrinsicMatrix';

proj = zeros(size(points));
projEst = zeros(12,2,size(points,3));

%get distortion parameters
p = cameraParams.TangentialDistortion;
if(length(p) < 2)
    p(2) = 0;
end
k = cameraParams.RadialDistortion;
if(length(k) < 3)
    k(3) = 0;
end

axis_length = .1;
axis = [0,0,0,1;
        axis_length, 0, 0, 1;
        0, axis_length, 0, 1;
        0, 0, axis_length, 1]';
err = nan(size(points,1),size(points,3));
%loop over each armPose
for i = 1:size(armPose,3)
    %transform chessboard points from the world into the image
    projected = [([1,0,0,0;0,1,0,0;0,0,1,0]*baseT*armPose(:,:,i)*gripT*worldPoints)';...
                 ([1,0,0,0;0,1,0,0;0,0,1,0]*baseT*axis)';...
                 ([1,0,0,0;0,1,0,0;0,0,1,0]*baseT*armPose(:,:,i)*axis)';...
                 ([1,0,0,0;0,1,0,0;0,0,1,0]*baseT*armPose(:,:,i)*gripT*axis)'];
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
    
    proj(:,:,i) = projected(1:end-12,:);
    projEst(:,:,i) = projected(end-11:end,:);
    
    %find error in projection
    err(:,i) = sum((proj(:,:,i) - points(:,:,i)).^2,2);
end

%remove invalid points
err = err(~isnan(err(:)));

%drop the largest values as outliers and take mean
err = sort(err(:));
err = mean(err(1:floor((inliers/100)*size(err,1))));

%return mean error in pixels
err = sqrt(err);