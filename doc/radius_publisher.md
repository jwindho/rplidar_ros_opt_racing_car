# Radius Publisher

You can find the code discussed here in the [src](/rplidar_ros_opt_race_car/src/) directory

We incorporated this publisher to facilitate the adjustment of angle limitations as required. With this feature, you can modify the angle limitation during the program's runtime without the need to repeatedly save the changes.

## Code description:

### radius_publisher_cmdline.cpp

```
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "radius_publisher");
    ros::NodeHandle nh;

    ros::Publisher radius_pub = nh.advertise<std_msgs::Float64MultiArray>("/radius", 1);

    ros::Rate loop_rate(10);

    float max_radius = 30.0f;
    float min_radius = 0.0f;

    if (argc == 3) {
        std::stringstream ss_max(argv[1]);
        ss_max >> max_radius;

        std::stringstream ss_min(argv[2]);
        ss_min >> min_radius;
    }

    while (ros::ok()) {
        std_msgs::Float64MultiArray radius;
        radius.data.push_back(max_radius);
        radius.data.push_back(min_radius);

        radius_pub.publish(radius);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

This code runs an infinite loop and publishes a message on the ROS topic *"/radius"* every 10 seconds. The message contains two floating-point numbers representing the **maximum** and **minimum** radius. 
If the node is called with two arguments representing the maximum and minimum radius, those values are used. 
Otherwise, default values of **30** and **0** are used. Other nodes can subscribe to this message to access this information and react accordingly.


The if statement checks if the node is called with two arguments representing the maximum and minimum radius. 
If yes, these arguments are converted to floating-point numbers and stored in the corresponding variables **max_radius** and **min_radius**.
In the while loop, a new *Float64MultiArray* is initialized, representing the maximum and minimum radius, and published.
