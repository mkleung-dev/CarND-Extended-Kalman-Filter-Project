# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, utilize a kalman filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Beside the Kalman filter and the extended Kalman filter with CV (**C**onstant **V**elocity) model required in the project. I have also implemented the unscented Kalman filter with CTRV (**C**onstant **T**urn **R**ate and **V**elocity magnitude) model. The result is shown in the final section.

## File Description

|Files|Description|
|:----|:----------|
|`src/main.cpp`|1. Communicates with the Term 2 Simulator receiving data measurements. <br /> 2. Call functions to run the Kalman filter.  <br /> 3. Call functions to calculate RMSE.|
|`src/FusionEKF.h` <br /> `src/FusionEKF.cpp`|Class to handle the extended Kalman filter.|
|`src/FusionUKF.h` <br /> `src/FusionUKF.cpp`|Class to handle the unscented Kalman filter (C).|
|`src/FusionKF.h` <br /> `src/FusionKF.cpp`|Generic class of FusionEKF and FusionUKF|
|`src/kalman_filter.h` <br /> `src/kalman_filter.cpp`|Implementation of the Kalman filter and the extended Kalman filter.|
|`src/unscented_kalman_filter.h` <br /> `src/unscented_kalman_filter.cpp`|Implementation of the unscented Kalman filter.|
|`src/tools.h` <br /> `src/tools.cpp`|Helper function to calculate RMSE and the Jacobian matrix.|
|`src/measurement_package.h`|Class for measurements.|

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Basic Run Instructions

|Command|Description|
|:------|:----------|
|`./ExtendedKF EKF`|Using extended Kalman filter with both radar and Lidar measurement.|
|`./ExtendedKF EKF Radar-Only`|Using extended Kalman filter with radar measurement only.|
|`./ExtendedKF EKF Laser-Only`|Using extended Kalman filter with Lidar measurement only.|
|`./ExtendedKF UKF`|Using unscented Kalman filter with both radar and Lidar measurement.|
|`./ExtendedKF UKF Radar-Only`|Using unscented Kalman filter with radar measurement.|
|`./ExtendedKF UKF Laser-Only`|Using unscented Kalman filter with Lidar measurement.|

## Result and Discussions

Several experiments were performed under different situations.

### Dataset 1

|Implementation|RMS Error of Position X|RMS Error of Position Y|RMS Error of Velocity X|RMS Error of Velocity X|
|:------|:------:|:------:|:------:|:------:|
|Extended Kalman filter|0.0975|0.0854|0.4185|0.4790|
|Extended Kalman filter with radar measurement only|0.2339|0.3357|0.5575|0.7430|
|Extended Kalman filter with Lidar measurement only|0.1886|0.1541|0.7396|0.4737|
|Unscented Kalman filter|0.0748|0.0844|0.3525|0.2407|
|Unscented Kalman filter with radar measurement only|0.2230|0.3041|0.4348|0.3681|
|Unscented Kalman filter with Lidar measurement only|0.1761|0.1475|0.6249|0.2655|

For Dataset 1, the unscented Kalman filter with CTRV model performed better than the extnded Kalman filter with CV model in all situtaions.

The Kalman filters with Lidar measurement performed better than the Kalman filter with radar measurement.

The Kalman filters with all types of measurement performed much better than the Kalman filter with either one of measurements.

### Dataset 2

|Implementation|RMS Error of Position X|RMS Error of Position Y|RMS Error of Velocity X|RMS Error of Velocity X|
|:------|:------:|:------:|:------:|:------:|
|Extended Kalman filter|0.0735|0.0975|0.5126|0.4699|
|Extended Kalman filter with radar measurement only|0.2437|0.3372|0.6682|0.7880|
|Extended Kalman filter with Lidar measurement only|0.1667|0.1404|0.5216|0.3028|
|Unscented Kalman filter|0.0919|0.0784|0.6652|0.3239|
|Unscented Kalman filter with radar measurement only|0.3847|0.3287|0.6522|0.3989|
|Unscented Kalman filter with Lidar measurement only|0.1667|0.1404|0.5216|0.3028|

For Dataset 2, it is quite strange that the unscented Kalman filter with CTRV model did not perform better than the extnded Kalman filter with CV model.

Similar to Dataset 1, The Kalman filters with Lidar measurement performed better than the Kalman filter with radar measurement.

Similar to Dataset 1, The Kalman filters with all types of measurement performed much better than the Kalman filter with either one of measurements.

