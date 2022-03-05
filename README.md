# Vision Algorithms for Mobile Robotics

Fully implementation of a monocular Visual Odometry pipeline.

#

Project rated with full marks (no bonus).

#

## Task

Implement a monocular Visual Odometry pipeline with the following features: initialization of 3D landmarks (bootstrap), keypoint tracking between two
frames, pose estimation using established 2D-3D correspondences, and triangulation of new landmarks.
Additional implemented bonus feature: n-view Bundle Adjustment.  

Since the VO is monocular and loop closure is not implemented the estimated trajectory could be not global coherent, but it remains local choerent.  

Further info: [Assignment.pdf](Assignment.pdf).  

Video examples:  

- Parking dataset (easy - speed 20x):  

https://user-images.githubusercontent.com/79461707/150541608-bc7e87cb-d32b-47a2-941b-c09638cda77e.mp4


&nbsp;

- KITTI 05 dataset (hard - speed 40x):  

https://user-images.githubusercontent.com/79461707/150541624-dcae3405-665b-43ab-9f32-ec732e6b422b.mp4



## Setup

Run the Visual Odometry from [main.m](main.m).  

The repository doesn't provide any dataset.
Inside the same folder as the 'main.m' file is stored, there has to be a folder called 'datasets' which contains the three datasets 
(so inside 'datasets' there have to be the folders 'kitti', 'malaga-urban-dataset-extract-07' and 'parking').
The desired dataset can be chosen by setting the parameter 'ds' in line 8.  

Additionally, the bonus feature (n-view bundle adjustment) can be (de-)activated by setting the parameter 'params.BA' in line 103.  

The script will run through the complete specified dataset and save the full trajectory and and landmark estimation inside the generated folder *results/[dataset]*.