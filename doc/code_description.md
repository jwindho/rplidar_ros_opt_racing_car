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
    const float angle_min_rad = angle_offset + M_PI - (reversed ? angle_max : angle_min);
    const float angle_max_rad = angle_offset + M_PI - (reversed ? angle_min : angle_max);
    
    const float angle_increment = (angle_max_rad - angle_min_rad) / (node_count - 1);
    const float time_increment = scan_time / (node_count - 1);
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = angle_min_rad;     
    scan_msg.angle_max = angle_max_rad;  
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

.

.

.
```
    sl_lidar_response_device_info_t devinfo;
    op_result = drv->getDeviceInfo(devinfo);
    bool scan_frequency_tunning_after_scan = false;

    if( (devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID){
        scan_frequency_tunning_after_scan = true;
    }
    //two service for start/stop lidar rotate
    ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
    ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

    if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
       //start RPLIDAR A serials rotate by pwm
        drv->setMotorSpeed(600);     
    }
```
First a structure **devinfo** is initialized to store device information. Then, it determines if the LIDAR model is a newer series model that supports scan frequency tuning after scanning. If the model is not, it starts the LIDAR rotation by setting the motor speed to **600 PWM**.

```
    LidarScanMode current_scan_mode;
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    } else {
        std::vector<LidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (SL_IS_OK(op_result)) {
            sl_u16 selectedScanMode = sl_u16(-1);
            for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == sl_u16(-1)) {
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                            iter->max_distance, (1000/iter->us_per_sample));
                }
                op_result = SL_RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if(SL_IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us        
        points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/scan_frequency);
        angle_compensate_multiple = points_per_circle/360.0  + 1;
        if(angle_compensate_multiple < 1) 
          angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;
        ROS_INFO("current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", current_scan_mode.scan_mode,(int)(1000/current_scan_mode.us_per_sample+0.5),max_distance, scan_frequency); 
    }
    else
    {
        ROS_ERROR("Can not start scan: %08x!", op_result);
    }
```
The code checks if a specific scan mode is requested by the user. 

First, the LiDAR sensor is started in the typical scan mode using the **startScan** method with the parameters *false* and *true* to indicate that the scan should not be forced and that the typical scan mode should be used. The **op_result** variable is used to store the result of this operation.

If a specific scan mode is requested by the user, the code first retrieves all the supported scan modes using the **getAllSupportedScanModes** method, and stores them in the **allSupportedScanModes** vector. The code then iterates through the **allSupportedScanModes** vector to find the requested scan mode, and sets the **selectedScanMode** variable to the ID of the found scan mode. If the requested scan mode is not found, it prints an error message.
If the LiDAR sensor is successfully started it calculates and sets necessary variables.


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


```
        if (op_result == SL_RESULT_OK) { 
            if(scan_frequency_tunning_after_scan){ //Set scan frequency(For Slamtec Tof lidar)
                ROS_INFO("set lidar scan frequency to %.1f Hz(%.1f Rpm) ",scan_frequency,scan_frequency*60);
                drv->setMotorSpeed(scan_frequency*60); //rpm 
                scan_frequency_tunning_after_scan = false;
                continue;
            }
            op_result = drv->ascendScanData(nodes, count);
```
If the **op_result** is equal to **SL_RESULT_OK**, which indicates a successful scan, the code proceeds to perform further operations. If the **scan_frequency_tunning_after_scan** variable is set to true, the lidar motor speed is set to a desired value indicated by **scan_frequency**, after which the loop is continued.


```
if (op_result == SL_RESULT_OK) {
                
                if (angle_compensate) {
                     // Filters the scan data by angle
                    int filtered_count = 0;
                    sl_lidar_response_measurement_node_hq_t filtered_nodes[8192];
                    for (int i = 0; i < count; i++) {
                        if (getAngle(nodes[i]) >= RAD2DEG(angle_min) && getAngle(nodes[i]) <= RAD2DEG(angle_max)) {
                            filtered_nodes[filtered_count++] = nodes[i];
                        }
                    }

                    // Apply angle correction to filtered scan data
                    const int angle_compensate_nodes_count = RAD2DEG(angle_max) * angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    std::vector<sl_lidar_response_measurement_node_hq_t> angle_compensate_nodes(angle_compensate_nodes_count);

                    for (int i = 0; i < filtered_count; i++) {
                        float angle = getAngle(filtered_nodes[i]);
                        int angle_value = static_cast<int>(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;

                        int start = angle_value - angle_compensate_offset;
                        int end = start + angle_compensate_multiple;
                        if (end > angle_compensate_nodes_count) end = angle_compensate_nodes_count;

                        for (int j = start; j < end; j++) {
                            angle_compensate_nodes[j] = filtered_nodes[i];
                        }
                    }

                    int compensate_count = 0;
                    sl_lidar_response_measurement_node_hq_t compensate_nodes[8192];
                    for (int i = 0; i < angle_compensate_nodes_count; i++) {
                        if (angle_compensate_nodes[i].dist_mm_q2 / 4.0f / 1000 > 0 && angle_compensate_nodes[i].dist_mm_q2 / 4.0f / 1000 < 8) {
                            compensate_nodes[compensate_count++] = angle_compensate_nodes[i];
                        }
                    }

                    // publish only corrected scan data                   
                    publish_scan(&scan_pub, compensate_nodes, compensate_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
                
                }
```
The following command initializes a variable **filtered_count** that is incremented after each filtered value. 
In the If command then all data are considered, which are in the value range.

This code section performs **angle compensation on filtered lidar scan data**. It first calculates the number of nodes required for angle compensation based on the maximum angle and compensation factor. Then, it creates a vector to store the nodes and initializes it with the required number of nodes. The for loop iterates over the filtered nodes, calculates the **angle value** and **offset**, and fills the vector with the corresponding node values. Afterward, another for loop iterates over the angle-compensated nodes, filters them based on distance, and fills the **compensate_nodes** array with the filtered nodes. And after that, only the filtered data will be published.

```
else {

                    int ANGLE_MIN = RAD2DEG(angle_min);
                    int ANGLE_MAX = RAD2DEG(angle_max);
                    const int MAX_NODES = 8192;
                    
                    sl_lidar_response_measurement_node_hq_t filtered_nodes[MAX_NODES];
                    int filtered_count = 0;
                    int start_node = 0, end_node = 0;
                    int i = 0;

                    // find the first valid node and last valid node
                   for(; i < count && (getAngle(nodes[i]) < ANGLE_MIN || getAngle(nodes[i]) > ANGLE_MAX); ++i) {}

                    start_node = i;

                    // find the last valid node
                    for(; i < count && getAngle(nodes[i]) <= ANGLE_MAX; ++i) {}

                    end_node = i - 1;

                    // filter nodes
                    for(int i = start_node; i <= end_node; ++i) {
                        const float distance = nodes[i].dist_mm_q2 / 4.0f / 1000;
                        if(distance > 0 && distance < 8) {
                            filtered_nodes[filtered_count++] = nodes[i];
                        }
                    }
                    publish_scan(&scan_pub, filtered_nodes, filtered_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
```
This code section finds the first (**start_node**) and last (**end_node**) valid nodes that fall within the angle range, filters the nodes based on distance in meters, and publishes the filtered data as a scan.
