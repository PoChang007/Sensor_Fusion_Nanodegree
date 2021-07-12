# 3D Object Tracking

<img src="images/course_code_structure.png" width="779" height="414" />
<hr>

In this project, we will implement the missing parts in the schematic based on what we have done so far in project 2. There are four major tasks in this projects: 
1. First, we will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, we will compute the TTC based on Lidar measurements. 
3. we will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, we will conduct various tests with the framework. Our goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.

[Project Writeup](final_project_writeup.md#Final-Project-WriteUp)

## Dependencies for Running Locally

* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.5
  * The OpenCV 4.5.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.5.0)
  * For the SIFT and SURF detectors, [OpenCV's extra modules](https://github.com/opencv/opencv_contrib) needs to be installed. See the build instruction below.
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo: `git clone https://github.com/PoChang007/Sensor_Fusion_Nanodegree.git`
2. `cd Sensor_Fusion_Nanodegree/Project_3_3D_Object_Tracking`
3. `mkdir build && cd build`
4. Compile: `cmake -DOPENCV_EXTRA_MODULES_PATH=/path_to_opencv_contrib/opencv_contrib/modules / -D WITH_VTK=OFF / -D BUILD_opencv_viz=OFF ..`
4. `make -j4`
4. Run it: `./3D_object_tracking`.
