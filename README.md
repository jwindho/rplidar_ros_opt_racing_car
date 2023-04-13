# Project description:  
In the following, the RPLidar A2 from Slamtec will be used to implement code using the C++ language.
For the final project, an RPLidar A3 will be used to support sensorics in an autonomous vehicle.
It should transfer data as quickly as possible in terms of performance, as it is intended to detect objects in its environment.
The code should run on Ubuntu 18.04.07 and communicate with ROS Melodic.


# Aims of the project: 
In addition to the fastest possible data transmission, the position of the lidar is not fixed precisely. Therefore, the code must be able to flexibly change its measurement angle.
For example, if the lidar is installed at the front of the car, only a certain radius is relevant for our data processing.
The primary goal of this project is to detect objects and thereby promote the fastest possible reaction speed of the vehicle.

A Main branch has been created for pulling the finished code. 

The opt-main branch, on the other hand, was created for understanding, testing and optimizing the code. Therefore, you will also find the documentation there.

# Main branch description: 

The actual code for starting the LIDAR sensor and reading the data can be found in the [src](src) folder. If you do not know how to start it or need further information about the code, help can be found in the doc-folder of the opt-main branch.





