# Performance-Code description:



## node_perf.cpp 

### Timer

```
class Timer {
public:
    Timer() : m_startTime(std::chrono::high_resolution_clock::now()) {}

    ~Timer() {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTime).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTime).time_since_epoch().count();
        auto duration = end - start;
        double ms = duration * 0.001;

        std::cout << "Elapsed time: " << ms << " ms\n";
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};

```

First the member variable **m_startTime** stores the current time. The destructor of the Timer class is called when the Timer object goes out of scope 
and uses the same function to get the current time, which is stored in the variable **endTime**. 
The elapsed time between the start and end times is calculated by subtracting the start time from the end time **duration** and converting the result to microseconds **ms**.



```
const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

float angle_min = DEG2RAD*(0.0f);
float angle_max = DEG2RAD*361.0f;
```
By setting a range from 0 to 361 degrees, the LIDAR sensor can scan the environment in a complete circle around its own axis, 
including the direction in which it points itself.



### publish_scan

```
void publish_scan(
                  NodeInfo nodes,
                  size_t node_count, double start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance
                  )
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

}
```
The function initializes a static variable called **scan_count** and increments it each time the function is called. 
It also sets two constants, **range_min** and **angle_offset**.
The code then calculates the *minimum* and *maximum angles* in radians, taking into account whether the angles should be reversed, 
and calculates the angle increment and time increment for the scan.


### main

```
const int angle_compensate_nodes_count = ANGLE_MAX* angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    NodeInfo angle_compensate_nodes(angle_compensate_nodes_count);

                    for (int i = 0; i < filtered_count ; i++) { 
                        float angle = getAngle(filtered_nodes.getAngle(i));
                        int angle_value = static_cast<int>(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;

                        int start = angle_value - angle_compensate_offset;
                        int end = start + angle_compensate_multiple;

                        if (end > angle_compensate_nodes_count) end = angle_compensate_nodes_count;

                        for (int j = start; j < end ; j++) { 
                            
                            angle_compensate_nodes.setAngle(j ,filtered_nodes.getAngle(i));
                            angle_compensate_nodes.setDistance(j, filtered_nodes.getDistance(i));

                        }
                    }

```
If the calculated angle is less than the current **angle_compensate_offset**, it is updated to equal the calculated angle. 
*start* and *end* values are calculated to indicate the range of indexes in **angle_compensate_nodes** to be updated with the current scan data point's angle and distance values.


```
                    int compensate_count = 0;
                    NodeInfo compensate_nodes(size_data);

                    for (int i = 0; i < angle_compensate_nodes_count; i++) {
                        if (angle_compensate_nodes.getDistance(i)  > 0 && angle_compensate_nodes.getDistance(i) < 800) {
                                                        
                            
                            
                            compensate_nodes.setDistance(compensate_count, angle_compensate_nodes.getDistance(i));
                            compensate_nodes.setAngle(compensate_count++, angle_compensate_nodes.getAngle(i)); 

                        }
                    }
```
The second part of the code initializes a new NodeInfo object called **compensate_nodes** that will store the compensated and filtered *angle* and *distance values*. 
A loop iterates through the **angle_compensate_nodes** object to filter out points that have a distance value less than or equal to 0 or greater than or 
in this case equal to 800. The valid angle and distance values are stored in **compensate_nodes**.



## node_info.cpp

```
#include "node_info.h"
#include <cstring>

NodeInfo::NodeInfo(int size) : size_(size) {
    memset(distance_, 0, size_ * sizeof(double));
    memset(angle_, 0, size_ * sizeof(double));
}

void NodeInfo::setDistance(double distances[]) {
    memcpy(distance_, distances, size_ * sizeof(double));
}

void NodeInfo::setDistance(int index,double distances ) {
    distance_[index] = distances;
}

void NodeInfo::setAngle(double angles[]) {
    memcpy(angle_, angles, size_ * sizeof(double));
}

void NodeInfo::setAngle(int index, double angles) {
    angle_[index] = angles;
}

double NodeInfo::getDistance(int index) {
    return distance_[index];
}

double NodeInfo::getAngle(int index) {
    return angle_[index];
}

```

The code defines a class which stores distance and angle information for a set of nodes as a simulation for data transmission without ros. 
It has a constructor that initializes the size of the node set, and methods for setting and getting the distance and angle values of individual nodes. 
The **setDistance** and **setAngle** methods have overloads for setting multiple values at once or setting individual values by index.





*A more comprehensive code description can be found here:* [Code description](https://github.com/jwindho/rplidar_ros_opt_racing_car/tree/opt-main/doc/code_description.md)
