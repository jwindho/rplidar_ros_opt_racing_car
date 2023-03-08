# Project description:  
In the following, the RPLidar A2 from Slamtec will be used to implement code using the C++ language.
For the final project, an RPLidar A3 will be used to support sensorics in an autonomous vehicle.
It should transfer data as quickly as possible in terms of performance, as it is intended to detect objects in its environment.
The code should run on Ubuntu 18.04.07 and communicate with ROS Melodic.


# Aims of the project: 
In addition to the fastest possible data transmission, the position of the lidar is not fixed precisely. Therefore, the code must be able to flexibly change its measurement angle.
For example, if the lidar is installed at the front of the car, only a certain radius is relevant for our data processing.
The primary goal of this project is to detect objects and thereby promote the fastest possible reaction speed of the vehicle.

# Setup Ubuntu & ROS Melodic
To be able to use ROS Melodic, we need to install the following Ubuntu environment: Ubuntu 18.04.07. 

This version is officially supported by ROS Melodic.

Now follow these instructions to set up ROS Melodic for your Ubuntu environment:
    http://wiki.ros.org/melodic/Installation/Ubuntu
    
# Creating a workspace
To successfully run the code, a separate workspace needs to be created where all necessary libraries and codes will be located later.

```
mkdir -p ~/catkin_ws/src
```

Navigate to the "src" directory within your workspace:

```
cd ~/catkin_ws/src
```

Initialize the workspace by running the catkin_init_workspace command:

```
catkin_init_workspace
```

Navigate back to the root of your workspace:

```
cd ~/catkin_ws
```

Build the workspace by running the catkin_make command:

```
catkin_make
```

Finally, activate your workspace by sourcing the setup script:

```
source ~/catkin_ws/devel/setup.bash
```

# Import the RPLIDAR ROS package into catkin_ws
Slamtec provides a default code that can both capture and output data. 
To create the necessary programming environment, we will use their source folder for the first time, which will then be modified.
Open a terminal and navigate to your catkin workspace src folder:

```
cd ~/catkin_ws/src
```

Clone the project repository:

```
git clone https://github.com/Slamtec/rplidar_ros.git
```

Navigate to the root of your catkin workspace:

```
cd ~/catkin_ws
```

Run the catkin_make command to build the workspace:

```
catkin_make
```

If the build completes successfully, activate your catkin workspace:

```
source ~/catkin_ws/devel/setup.bash
```

# Run Code
To address the correct port, it first needs to be determined to which port the Lidar is connected.

```
ls -l /dev |grep ttyUSB
```

Add the authority of write: (such as /dev/ttyUSB0)

```
sudo chmod 666 /dev/ttyUSB0
```

To start the launcher, you must now call the following command:

```
roslaunch rplidar_ros rplidar.launch
```

Alternatively, you can display the data via the terminal window. The command for this is:

```
rosrun rplidar_ros rplidarNodeClient
```

# Code description:



## Ros-Client 


### scanCallback()

```
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}
```
This **scanCallback** function is executed when a new LaserScan message is received. This function extracts 
the information from the message and outputs it to the console. First the number of measurements *count* is 
calculated from the **scan time** and the measurement **time_increment**. Then the frame name, the number of 
measurements and the angular range of the measurement in degrees is output to the console. Finally, the 
measurements are iterated over in a loop and the angles and distances of each measurement are output in 
degrees and metres on the console.



### int(main)

```
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
```

The function main initialises the ROS node named **rplidar_node_client** and creates a **NodeHandle n** to communicate 
with the ROS system. Then a subscriber sub is created that listens on the message source */scan* and calls the **scanCallback** 
function when a message is received. The number 1000 indicates the maximum number of messages held in the queue. Finally, 
the ROS node is sent into a continuous loop to wait for incoming messages until the programme is terminated.



## Ros-Node 

### publish_scan()

The **publish_scan** function has parameters for configuring the laser scan message, including the minimum and maximum 
angles for the scan data, the maximum distance and the timestamp for the measurements. The function then calculates the 
angle range for the laser scan data and fills in the **ranges** and **intensities** fields in the LaserScan message based on
the measurements received.


### getRPLIDARDeviceInfo

The function **getRPLIDARDeviceInfo** retrieves information about a RPLIDAR device, formats this information and outputs it.
The function returns *true* if the information was retrieved successfully, otherwise it returns *false*. The information that is 
output includes the serial number of the unit, the firmware and hardware versions, and the model name of the unit.

```
char sn_str[35] = {0}; 
for (int pos = 0; pos < 16 ;++pos) {
    sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
}
char mode_str[16] = {0};
if((devinfo.model>>4) <= LIDAR_S_SERIES_MINUM_MAJOR_ID){
    sprintf(mode_str,"A%dM%d",(devinfo.model>>4),(devinfo.model&0xf));
} else if((devinfo.model>>4) <= LIDAR_T_SERIES_MINUM_MAJOR_ID){
    sprintf(mode_str,"S%dM%d",(devinfo.model>>4)-LIDAR_S_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));
} else{
    sprintf(mode_str,"T%dM%d",(devinfo.model>>4)-LIDAR_T_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));
}
```

These lines format the serial number of the RPLIDAR unit as a hexadecimal number and store it in the variable **sn_str**. 
They also format the model name of the unit and store it in the variable "mode_str".



### checkRPLIDARHealth

This code checks the state of a RPLIDAR laser scanner by calling the **getHealth** method of the **ILidarDriver** object 
and checking the return value. If the state of the scanner is *OK* or *Warning*, the function outputs a corresponding 
message and returns *true*. If the state is *Error*, the function outputs an error message and returns *false*. If the 
call to the *getHealth* method fails, the function also issues an error message and returns *false*.




### while(ros::ok) 

```
start_scan_time = ros::Time::now();
op_result = drv->grabScanDataHq(nodes, count);
end_scan_time = ros::Time::now();
scan_duration = (end_scan_time - start_scan_time).toSec();
```

The **start_scan_time** and **end_scan_time** are stored here in order to calculate the duration of the scan data later. 
The command **drv->grabScanDataHq(nodes, count)** ensures that the scan data is received from the laser scanner and 
stored in the array nodes. The return value **op_result** indicates whether the scan process was successful or not. 
Finally, the duration of the scan data is calculated in seconds and stored in the variable **scan_duration**.

