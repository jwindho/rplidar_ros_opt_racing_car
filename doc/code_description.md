# Code description:



## Ros | client.cpp 


### scanCallback

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
This **scanCallback** function is executed when a new LaserScan message is received. The function extracts information from the message and outputs it to the console. By calculating **count**, the number of scan data per scan is determined. This information can be used to determine the accuracy and resolution of the point cloud captured by the LIDAR sensor. First the number of measurements **count** is 
calculated from the *scan_time* and the measurement *time_increment*. Then the frame name, the number of 
measurements and the angular range of the measurement in degrees is output to the console. Finally, the 
measurements are iterated over in a loop and the angles and distances of each measurement are output in 
degrees and meters on the console.


### int main

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

The function **main** initialises the ROS node named *rplidar_node_client* and creates a **NodeHandle n** to communicate 
with the ROS system. Then a subscriber **sub** is created that listens on the message source */scan* and calls the *scanCallback* 
function when a message is received. The number 1000 indicates the maximum number of messages held in the queue. Finally, the **ros::spin()** function is called to keep the node running and to handle incoming messages from the subscribed topics until the programme is terminated. 



## Ros | node.cpp 

### publish_scan

```
void publish_scan(ros::Publisher* pub,
    sl_lidar_response_measurement_node_hq_t* nodes,
    size_t node_count, ros::Time start,
    double scan_time, bool inverted,
    float angle_min, float angle_max,
    float max_distance,
    std::string frame_id)
{
    static int scan_count = 0;
    const float range_min = 0.15f;
    const float angle_offset = M_PI;
    const bool reversed = angle_max > angle_min;

    scan_count++;
    //Unnötig ?
    const float angle_min_rad = angle_offset + M_PI - (reversed ? angle_max : angle_min);
    const float angle_max_rad = angle_offset + M_PI - (reversed ? angle_min : angle_max);
    //
    const float angle_increment = (angle_max_rad - angle_min_rad) / (node_count - 1);
    const float time_increment = scan_time / (node_count - 1);
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = angle_min_rad;     //was bringt mir diese Berechnung und information ?
    scan_msg.angle_max = angle_max_rad;     //
    scan_msg.angle_increment = angle_increment;
    scan_msg.time_increment = time_increment;
    scan_msg.scan_time = scan_time;
    scan_msg.range_min = range_min;
    scan_msg.range_max = max_distance;
    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    for (size_t i = 0; i < node_count; ++i) {
        const float read_value = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;

        const size_t index = (inverted != reversed) ? (node_count - 1 - i) : i;
        scan_msg.ranges[index] = read_value;
        scan_msg.intensities[index] = nodes[i].quality >> 2;
    }
    pub->publish(scan_msg);
}
```

The **publish_scan** function has parameters for configuring the laser scan message, including the minimum and maximum 
angles for the scan data, the maximum distance and the timestamp for the measurements. The function then calculates the 
angle range for the laser scan data and fills in the **ranges** and **intensities** fields in the LaserScan message based on
the measurements received. It iterates through each measurement point in the input array, converts the distance value from millimeters to meters (**read_value**), and populates the **ranges** and **intensities** arrays of the LaserScan message object with the converted values.


### getRPLIDARDeviceInfo

```
bool getRPLIDARDeviceInfo(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            ROS_ERROR("Error, unexpected error, code: %x",op_result);
        }
        return false;
    }
    
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
    ROS_INFO("RPLIDAR MODE:%s",mode_str);
    ROS_INFO("RPLIDAR S/N: %s",sn_str);
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}
```
The function **getRPLIDARDeviceInfo** retrieves information about a RPLIDAR device, formats this information and outputs it.
The function returns *true* if the information was retrieved successfully, otherwise it returns *false*. If the call to **drv->getDeviceInfo(devinfo)** is successful, the function then converts the device serial number and model number from hexadecimal to ASCII string format using **sprintf** function. The serial number is stored in **sn_str** and the model number is stored in **mode_str**.
The information output includes the serial number of the unit, the firmware/hardware versions, and the model name of the unit.


### checkRPLIDARHealth

```
bool checkRPLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { 
        //ROS_INFO("RPLidar health status : %d", healthinfo.status);
        switch (healthinfo.status) {
			case SL_LIDAR_STATUS_OK:
                ROS_INFO("RPLidar health status : OK.");
				return true;
			case SL_LIDAR_STATUS_WARNING:
                ROS_INFO("RPLidar health status : Warning.");
				return true;
			case SL_LIDAR_STATUS_ERROR:
                ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
				return false;
        }
    } else {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}
```

This code checks the state of a RPLIDAR laser scanner by calling the **getHealth** method of the **ILidarDriver** object 
and checking the return value. If the **getHealth** function call is successful *SL_IS_OK(op_result)*, the function prints the device health status as *OK*, *Warning*, or *Error* using a switch statement based on the **healthinfo.status** value and returns true for *OK* and *Warning* statuses, and false for *Error* status. If the 
call to the *getHealth* method fails, the function also issues an error message and returns *false*.

### stop_motor

```
bool stop_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;

  ROS_DEBUG("Stop motor");
  drv->setMotorSpeed(0);
  return true;
}
```
The function first checks if the variable **drv** is *true*. If it is, the function sets the motor speed to 0 using the **setMotorSpeed** function.
Else the function returns *false*. It also logs a debug message using **ROS_DEBUG**.

### start_motor

```
bool start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;
  if(drv->isConnected())
  {
      ROS_DEBUG("Start motor");
      sl_result ans=drv->setMotorSpeed();
  
      ans=drv->startScan(0,1);
   }
   else ROS_INFO("lost connection");
  return true;
}
```
Again, the function checks whether **drv** is *true* or *false*. If the query is *true*, the code checks if there is a connection using the **isConnected** function. If it is connected, the function logs a debug message using ROS_DEBUG, and then calls the **setMotorSpeed** function of the object to set the motor speed. The function then calls the **startScan** function of our object to set the motor speed and to run the motor.


### getAngle
```
static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}
```
This code defines a static function called **getAngle** that takes in a constant reference to our struct and calculates the **angle** in degrees represented by the given node in the lidar measurement data.


### radius_callback

```
void radius_callback(const std_msgs::Float64MultiArray& msg) {
    if (msg.data.size() == 2) {
        angle_max = DEG2RAD(msg.data[0]);
        angle_min = DEG2RAD(msg.data[1]);
    }
}
```

This code takes in a message of type *Float64MultiArray*. Within the function it sets two variables called "angle_max" and "angle_min" to the values contained in the first and second elements of the "data" array respectively, after converting them from degrees to radians using the "DEG2RAD" function.

### int main

The code initializes the ROS node **rplidar_node** and sets up parameters for connecting to a RPLIDAR sensor via different channels *(serial, TCP, UDP)*. It also sets some sensor-specific parameters, such as whether the sensor data needs **angle compensation** and the **maximum scan distance**. The code subscribes to a ROS topic */radius* to adjust the angle to be measured. It creates a publisher for publishing the sensor data as a **sensor_msgs/LaserScan** message to the **scan** topic. It creates a driver instance for the RPLIDAR and connects to the sensor via the specified channel type and parameters. If the connection fails, an error message is printed, and the program terminates with a return code of -1.

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
