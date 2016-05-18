Camera-to-Arm-Calibration
=========================

The method operates by finding the parameters that minimize the difference in the position of the checkerboard given by the camera and arm when it is projected into the cameras coordinate system.

A ROS extension to this toolbox that removes the need for manual extraction of data has been made by Haitham El-Hussieny and can be found at https://github.com/elhussieny/kuka_camera_calib

The details of how the calibration works are given in Camera to Robotic Arm Calibration.pdf
 
Calibration guide
=================

1) Download the code

2) Obtain a checkerboard, Matlab can generate one by running 
	
	I = checkerboard(200,10,7); I = I(1:size(I,1)/2,1:size(I,2)/2); imsave(imshow(I));

3) Rigidly mount the checkerboard to the robots end effector

4) Move the robotic arm into a new pose where the entire checkerboard can be seen from the camera

5) Capture the camera image and record the arm base to end effector transformation.

6) Repeat steps 4-5 at least 10 times (20+ times is recommended)

7) Run the calibration code

	[TBase, TEnd, cameraParams] = CalCamArm(imageFolder, armMat, squareSize)
		imageFolder- folder containing images
		armMat- matrix holding arm base to end effector transformations
		squareSize- width of a square on the chessboard in mm
		TBase- 4x4 camera to arm base transformation matrix, distances in m
		TEnd- 4x4 arm base to end effector transformation matrix, distances in m
		cameraParams- camera parameters object containing distortions and camera matrix
	
8) The Calibration should be complete. For more details and optional inputs / outputs run

	help CalCamArm

Any issues / comments / ideas / etc give me an email at z.taylor@acfr.usyd.edu.au
