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
