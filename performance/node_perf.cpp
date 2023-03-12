

 
#include "node_info.h"
#include <iostream>
#include <string>

using namespace std;

# define M_PI           3.14159265358979323846

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif






const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

float angle_min = DEG2RAD*0.0f;
float angle_max = DEG2RAD*360.0f;

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
    //Unnötig ?
    const float angle_min_rad = angle_offset + M_PI - (reversed ? angle_max : angle_min);
    const float angle_max_rad = angle_offset + M_PI - (reversed ? angle_min : angle_max);
    //

    const float angle_increment = (angle_max_rad - angle_min_rad) / (node_count - 1);


    const float time_increment = scan_time / (node_count - 1);

    /*
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = angle_min_rad;     //was bringt mir diese Berechnung und information ?
    scan_msg.angle_max = angle_max_rad;     
    scan_msg.angle_increment = angle_increment;
    scan_msg.time_increment = time_increment;
    scan_msg.scan_time = scan_time;
    scan_msg.range_min = range_min;
    scan_msg.range_max = max_distance;
    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    */
    
    

    for (size_t i = 0; i < node_count; ++i) {
        
        cout << "Angle = " << nodes.getAngle(i) << "    Distance = " << nodes.getDistance(i) << endl; 
    }

  
}








static float getAngle(double node)
{
    return RAD2DEG * node  ;
}

//Callback Funktion um den Max und Min Radius zu setzten 



/**/
int main() {
    
    const int size_data = 4;

    NodeInfo nodes(size_data); // Größe der Arrays ist 8192
    
    double distances[size_data] = {5.0, 7.0, 1.0, 3.0 };
    nodes.setDistance(distances);
    
    double angles[size_data] = {0, M_PI/2 , M_PI , 2*M_PI};
    nodes.setAngle(angles);


    std::string channel_type;
    std::string tcp_ip;
    int tcp_port = 20108;
    std::string udp_ip;
    int udp_port = 8089;
    std::string serial_port;    
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;    
    float angle_compensate_multiple = 1.0;//min 360 ponits at per 1 degree
    int points_per_circle = 360;//min 360 ponits at per circle 
    std::string scan_mode;
    float max_distance;
    double scan_frequency;
    


    

    double scan_duration;
    int loop = 0;
    
    while (loop < 1) {
       
        size_t   count = size_data;

        
        
        double start_scan_time = 0;
        double end_scan_time = 100;
        scan_duration = (end_scan_time - start_scan_time);
        angle_compensate = true; 


                if (angle_compensate) {
                     // Filtere die Scan-Daten nach Winkel

                    int filtered_count = 0;
                    NodeInfo filtered_nodes(size_data);
                    for (int i = 0; i < count; i++) {
                        if (getAngle(nodes.getAngle(i)) >= RAD2DEG*angle_min && getAngle(nodes.getAngle(i)) <= RAD2DEG*angle_max) {
                            
                            
                            filtered_nodes.setAngle(filtered_count++, nodes.getAngle(i));
                            filtered_nodes.setDistance(filtered_count++, nodes.getDistance(i));
                            
                            
                        }
                    }

                    // Winkelkorrektur auf gefilterte Scan-Daten anwenden
                    const int angle_compensate_nodes_count = RAD2DEG*angle_max * angle_compensate_multiple;
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

                    int compensate_count = 0;
                    NodeInfo compensate_nodes(size_data);
                    for (int i = 0; i < angle_compensate_nodes_count; i++) {
                        if (angle_compensate_nodes.getDistance(i) / 4.0f / 1000 > 0 && angle_compensate_nodes.getDistance(i) / 4.0f / 1000 < 8) {
                            
                            
                            
                            
                            compensate_nodes.setDistance(compensate_count++, angle_compensate_nodes.getDistance(i));
                            compensate_nodes.setAngle(compensate_count++, angle_compensate_nodes.getAngle(i)); //

                        }
                    }

                    // veröffentliche nur die korrigierten Scan-Daten
                   
                    publish_scan(compensate_nodes, compensate_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                
                
                } else {

                    int ANGLE_MIN = RAD2DEG*angle_min;
                    int ANGLE_MAX = RAD2DEG*angle_max;
                    const int MAX_NODES = size_data;
                    
                    NodeInfo filtered_nodes(MAX_NODES);
                    int filtered_count = 0;
                    int start_node = 0, end_node = 0;
                    int i = 0;

                    // find the first valid node and last valid node
                   for(; i < count && (getAngle(nodes.getAngle(i)) < ANGLE_MIN || getAngle(nodes.getAngle(i)) > ANGLE_MAX); ++i) {}

                    start_node = i;

                    // find the last valid node
                    for(; i < count && getAngle(nodes.getAngle(i)) <= ANGLE_MAX; ++i) {}

                    end_node = i - 1;

                    // filter nodes
                    for(int i = start_node; i <= end_node; ++i) {
                        const float distance = nodes.getDistance(i) / 4.0f / 1000;
                        if(distance > 0 && distance < 8) {


                            filtered_nodes.setAngle(filtered_count++, nodes.getAngle(i));
                            filtered_nodes.setDistance(filtered_count++, nodes.getDistance(i));

                       
                       
                        }
                    }
                    publish_scan(filtered_nodes, filtered_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                    }
            loop ++  ;



            } 
        

        
    

    
    
    return 0;
}
