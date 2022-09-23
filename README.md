# Edge_detection
[SOLVED] Challenge on edge_detection in ROS

[IMPORTANT] Need to add the robot package which has the robot models and the bag file in the src folder and also have opencv installed in the system for building this project. Not uploaded here due to file size issues. The other dependencies can be found in the ``` CMakeLists.txt ```.

## Work-flow
1) Implement canny edge detection provided by opencv on the given image and then perform the hough line transform to plot edges as green lines on the image.

2) These edges in pixel coordinates should then be converted to 3D point cloud using Perspective Projection Model and then visualised in RViz along with the robot when the bag file is played.

3) The final result of the point cloud can be seen in the videos section. Also, an intermediate result of visualising images in RViz with green lines as edges from images taken from a topic published by the bag file is also provided.

## Description of code
--> The ``` edge_detection ``` package has 1 C++ class file ``` EdgeDetector.cpp / .h ``` and 2 C++ source files ``` Robocop.cpp ``` & ``` EdgeDetectService.cpp ``` and these two are the executables you can find. The code has been made as modular & readable as possible in the given time.

--> The executable ``` EdgeDetectService.cpp ``` implements the detection by a service call method. The service file is written in package ``` edge_srv ```. An ``` edge_msg ``` package has also been initialised in case of requirement for custom messages.

--> The ``` EdgeDetector ``` class files have all the methods for implementing edge detection, line drawing, and 3D point cloud generation.

--> The executable ``` Robocop.cpp ``` is the main node to be run, which subscribes to the color_img, depth_img and camera_info published by the bag file and then performs 2 operations and publishes the results on their respective topics. The 2 tasks are :

(i) Perform edge detection for images subscribed from bag file and then draw green lines for the edges. The result images are then published to a topic ``` /detectedEdges ``` which can be visualised in RViz. Working video provided.

(ii) Perform edge detection as above and then convert these edge pixels to 3D point clouds using the Perspective Projection Model for a simple pin-hole camera and then publishing it to ``` /edge_points ```. This can then be visualised in the RViz window when you launch the Mira robot. The bag file should be played to see the robot move and visualise the point clouds accordingly. Working video provided.

## Notable Challenges
* The service call implementation of the edge detection doesn't work as expected. Although no build issues, there might have been some inadverdent discrepancy which I believe can be fixed by combing through the code.
* Environment setup and dependencies has taken quite some time in the initial stages with build errors popping up all the time.
* Although we can visualise the final 3D pointCloud result in RViz, I think the results can be better using other better methods & techniques. 

## Conclusion

This project is supposed to test the fundamentals of ROS and C++ along with computer vision. I believe that the results produced and the code developed satisfy the requirements.
Nonetheless, I had great fun and a memorable learning experience while working on this challenge.

## References

* Computer Vision lectures by Prof. Vincent Fremont, ROS & software architeture course by Prof. Gaeten Garcia, ROS - vision, manipulation and sensor based control by Prof. Olivier Kermorgant at Ecole Centrale de Nantes.
* ROS & OpenCV documentation
* Open Source forums on ROS, C++ & OpenCV
