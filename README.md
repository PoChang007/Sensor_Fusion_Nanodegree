# Udacity - Sensor Fusion Nanodegree Program

The summary of my learnings in Udacity's SFND program.

## Installation

1. Clone this repo: `git clone https://github.com/PoChang007/Sensor_Fusion_Nanodegree.git`
2. Install Git [LFS](https://git-lfs.github.com/)

## Build Instructions

See the build instructions in each project folder.

## [Project 1 - Lidar Obstacle Detection](https://github.com/PoChang007/Sensor_Fusion_Nanodegree/tree/main/Project_1_Lidar_Obstacle_Detection)

In this project, we use Lidar Point Cloud to detect the obstacle on the road. The RANSAC algorithm is used to segment out the ground and the scene from the 3D data. The KD-tree algorithm is applied to cluster point clouds such that obstacles in the scene can be identified. To speed up the obstacle detection processing, we do point cloud filtering by downsampling lidar data and cropping out the region of interest within the defined distance from the ego car.

## [Project 2 - 2D Feature Tracking](https://github.com/PoChang007/Sensor_Fusion_Nanodegree/tree/main/Project_2_2D_Feature_Tracking)

In this project, we use the camera to extract the features of preceding vehicles by applying well-known feature tracking algorithms. The keypoints will be first detected. Then, the descriptors for each keypoint can be extracted. The matching algorithms are used to match the corresponding keypoints detected in the incoming frame. The purpose of obtaining feature points from preceding vehicles is for time-to-collision (TTC) estimation in the third project. The evaluation on keypoint searching accuracy and processing speed for each algorithm is also conducted.

## [Project 3 - 3D Object Tracking (Lidar/Camera)](https://github.com/PoChang007/Sensor_Fusion_Nanodegree/tree/main/Project_3_3D_Object_Tracking)

In this project, we combine both lidar and camera data for 3D object tracking. An object detection algorithm is used to first identifying the region of interest (ROI) in the color image. Then the associated keypoint correspondences are used to match 3D objects over time. The time-to-collision (TTC) can be estimated based on lidar 3D points and keypoint matching pairs within the ROIs.

## [Project 4 - Radar](https://github.com/PoChang007/Sensor_Fusion_Nanodegree/tree/main/Project_4_Radar_Target_Generation_And_Detection)

In this project, we first configure the FMCW waveform based on the radar system requirements. Then, we define the range and velocity of target and simulate its displacement, and process the transmit and receive signal to determine the beat signal. After that, we perform range of FFT on the received signal to determine the range. Finally, we perform the CFAR processing on the output of second FFT to display the target.

## [Project 5 - Unscented Kalman Filter (Lidar/Radar)](https://github.com/PoChang007/Sensor_Fusion_Nanodegree/tree/main/Project_5_Unscented_Kalman_Filter)

In this project we implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.
