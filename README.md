# Visual Servoing

## Goal
The goal of this project is to control the position of a motor using a camera.

In particular, the target position will be set with a click of the mouse.

## Hardware
- ESP32 devkit V1 
- GBM5208-75T brushless motor
- SimpleFOC shield mini
- AS5600 magnetic encoder
- 15V power supply
- a computer running the computer vision program

![Image](setup.jpg)

## Software

### Microcontroller
The microcontroller runs a simple velocity control program. The control loop is almost entirely handled with the SimpleFOC library, that allows for a simple and effective integration of the BLDC driver, the magnetic sensor, and the PID controller.

The microcontroller is always listening to the serial commands sended by the PC. Specifically it uses the commander interface (provided by the SimpleFOC library) to get the velocity setpoint to track.

The controller is a PI, hand-tuned, with a speed limit to allow the camera to correctly detect the elements without eccessive blur.

### Computer vision
The computer vision part is written in python, with the OpenCV library.

To track the position of the motor, I designed and 3D printed a little bracket that is attached to the rotor. At the tip of this bracket I taped a little green circle made of paper.

The vision algorithm is summarized in these steps:
- acquire the new frame
- detect the green circle
    - convert to the HSV color space
    - apply a thresholding to select the green pixels
    - apply the Hough transform to detect circles (with the correct setup only one circle is detected)
- detect the center of the rotor with an analogous procedure
- calculate the angle of the rotor using the centers of these two circles

Then the user can define the setpoint to track by clicking directly on the captured image

The reference velocity is sent to the microcontroller via serial

## Results
The motor is able to track the reference signal defined by the user

[Video](https://imgur.com/r4XPlxJ)

---
# Camera calibration
Using a chessboard it is possible to measure the ditortion parameters of the camera and of the lens.

To make the project a bit more challengng, I added a wide-angle lens to the smartphone I use as camera

## Calibration process
I took 10 photos of the chessboard from various angles in order to capture most of the distortion

Then using the camera_calibration script I calculated the camera matrix and the distortion coefficients and saved them in a file

These values can be used to undistort any image taken with the camera, as we can see in the following images:

![Image](camera_calibration/PXL_20240302_230833618.jpg)
![Image](camera_calibration/calibresult.png)

# Perspective transformation
In order to apply the Hough transform even when the camera is pointed at an angle I need to correct the perspective of the image. Otherwise the circles are projected into ellipses and are no longer recognizable by the Hough transform.

To do that I added a permanent chessboard to the setup, positioned in a way such that it lies on the same plane as the circles to detect.

Then I defined a "target perspective" where the chessboard is in a specified position in the image plane (so that the squares are represented as perfect squares).

Using the fucntions provided by OpenCV, I detected the corners of the chessboard on both the distorted image and the target image, and then calculated the homography matrix. The homography matrix is then used to transform the distorted image in order to match the corners of the chessboard, producing a frame where the circles are seen as (almost) perfect circles
