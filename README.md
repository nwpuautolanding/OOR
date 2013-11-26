OOR
===

Vibot 2013 Robotics Project: Office Object Recognition

The aim of this project is to be able to recognize common office objects (tables, chairs, etc) using a Turtlebot and a Kinect.

Approach:
--------
- Merge point clouds comming from the Kinect into one big sceene using Octomap.
- Cluster data with Pcl plane segmentation or Real-Time Plane Segmentation using RGB-D Cameras (Dirk Holz, Stefan Holzer, Radu Bogdan Rusu and Sven Behnke)
- Recognize primitives (planes, cylinders)
- Graph matching, a table is deffined with 4 cylinders connected to a plane.


Libraries:
---------
- OpenCv
- Pcl
- OpenNi
- ROS

