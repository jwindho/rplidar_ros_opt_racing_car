# Project description:  
A set of a lidar sensor for a self-driving race car. 
This is to detect objects that are in front of the car on the track. 

## Aims of the project: 
Limit the radius of the RPLidar to 30 degrees, 
and optimise the performance of the code.


### Code description:

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

