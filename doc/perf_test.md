# Performance documentation

# ROS Performance 
The default code shows that the lidar transfers its data with a bandwidth of 35.01 KB/s and a frequency of 11.84 Hz.
After some initial changes, such as adjusting the angle from 360° to 30° (specified here in radians),
we can see that the bandwidth has decreased to 18.26 KB/s and the frequency has increased to 12 Hz.
Normally, a higher bandwidth means higher accuracy and faster response time, but since we have minimized the necessary measuring radius,
less computing power is needed to transmit the data. The increased frequency is an indication of improved performance for now.
Another indicator of improved performance is the scan_time, which has been reduced from ~0.08742s to ~0.07997s.
Further changes to the code will follow.

# Default

![benchmarkin_default](https://user-images.githubusercontent.com/84909827/223700445-5bda0c12-77ad-4d9f-90c9-4611ccebeaf3.PNG)


# Changed

![benchmarking_v1](https://user-images.githubusercontent.com/84909827/223700629-387e3d76-2d24-4b9e-abc5-66c682005718.PNG)


# Code Performance

*The code being discussed is located in the* [performance](/rplidar_ros_opt_racing_car/performance/)
*directory*

Our primary goal is to evaluate the code's performance, focusing specifically on its ability to process the files sent by the Lidar. To accomplish this, we removed all superfluous ROS applications and pruned code sections that only run once during start-up. We simulated the Lidar data packet with a custom class called "node_info" to test the code without having to connect the Lidar sensor.

We can create an instance of the ["node_info"](/rplidar_ros_opt_racing_car/performace/node_info.cpp) class in the main program and supply it with data to test the performance of the actual data processing. By utilizing a timer to measure the exact amount of time it takes for the program to execute once, we can assess the code's efficiency.

We made optimizations to the code and then integrated it into the primary program. The specific modifications can be observed in the commits.
