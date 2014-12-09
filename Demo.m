%Simple script to quickly demo use of CalCamArm.m

%path to images of checkerboards
imageFolder = './Example Data/Images/';
%loading arm transformations
load('./Example Data/armMat.mat');
%checkerboard square widths in mm
squareSize = 13.6;

%run calibration
[TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArm(imageFolder, armMat, squareSize,'maxBaseOffset',0.5);

%print results
fprintf('\nFinal camera to arm base transform is\n')
disp(TBase);

fprintf('Final end effector to checkerboard transform is\n')
disp(TEnd);

fprintf('Final camera matrix is\n')
disp(cameraParams.IntrinsicMatrix');

fprintf('Final camera radial distortion parameters are\n')
disp(cameraParams.RadialDistortion);

fprintf('Final camera tangential distortion parameters are\n')
disp(cameraParams.TangentialDistortion);