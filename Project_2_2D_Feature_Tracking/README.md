# 2D Feature Tracking

# [Project Writeup](project_2_writeup.md#Project-2-WriteUp)

<img src="images/keypoints.png" width="820" height="248" />
<hr>

The idea of this camera project is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, we will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This project consists of four parts:

* First, we will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, we will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, we will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, we will test the various algorithms in different combinations and compare them with regard to some performance measures.

## Dependencies for Running Locally

* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.5
  * The OpenCV 4.5.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.5.0)
  * For the SIFT and SURF detectors, [OpenCV's extra modules](https://github.com/opencv/opencv_contrib) needs to be installed. See the build instruction below.
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo: `git clone https://github.com/PoChang007/Sensor_Fusion_Nanodegree.git`
2. `cd Sensor_Fusion_Nanodegree/Project_2_2D_Feature_Tracking`
3. `mkdir build && cd build`
4. Compile: `cmake -DOPENCV_EXTRA_MODULES_PATH=/path_to_opencv_contrib/opencv_contrib/modules / -D WITH_VTK=OFF / -D BUILD_opencv_viz=OFF ..`
4. `make -j4`
5. Run it: `./2D_feature_tracking`.