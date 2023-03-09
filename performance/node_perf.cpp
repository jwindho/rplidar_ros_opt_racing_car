/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "sl_lidar.h" 
#include "node_info.h"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180/M_PI)

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID      = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID       = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID       = 8,
};
using namespace sl;

float angle_min = DEG2RAD(0.0f);
float angle_max = DEG2RAD(30.0f);

void publish_scan(
                  NodeInfo nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
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
        const float read_value = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;

        const size_t index = (inverted != reversed) ? (node_count - 1 - i) : i;
        scan_msg.ranges[index] = read_value;
        scan_msg.intensities[index] = nodes[i].quality >> 2;
    }

    
}








static float getAngle(double node)
{
    return node * 90.f / 16384.f;
}

//Callback Funktion um den Max und Min Radius zu setzten 




int main(int argc, char * argv[]) {
    

    NodeInfo node_info(8192); // Größe der Arrays ist 8192
    
    double distances[8192] = {5.0, 10.0};
    node_info.setDistance(distances);
    
    double angles[8192] = {30.0, 45.0};
    node_info.setAngle(angles);


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
    

    int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
    int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
    int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;    
    

    double scan_duration;
    int loop = 0;
    
    while (loop < 100) {
        float nodes_distance[];
        float node_angle[];
        size_t   count = _countof(nodes);

        
        start_scan_time = 0;
        end_scan_time = 100;
        scan_duration = (end_scan_time - start_scan_time).toSec();


                if (angle_compensate) {
                     // Filtere die Scan-Daten nach Winkel
                    int filtered_count = 0;
                    NodeInfo filtered_nodes[8192];
                    for (int i = 0; i < count; i++) {
                        if (getAngle(nodes.getAngle(i)) >= RAD2DEG(angle_min) && getAngle(nodes.getAngle(i)) <= RAD2DEG(angle_max)) {
                            
                            
                            
                            
                            
                            filtered_nodes[filtered_count++] = nodes[i];
                        }
                    }

                    // Winkelkorrektur auf gefilterte Scan-Daten anwenden
                    const int angle_compensate_nodes_count = RAD2DEG(angle_max) * angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    NodeInfo angle_compensate_nodes(angle_compensate_nodes_count);

                    for (int i = 0; i < filtered_count; i++) {
                        float angle = getAngle(filtered_nodes.getAngle(i));
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
                    NodeInfo compensate_nodes[8192];
                    for (int i = 0; i < angle_compensate_nodes_count; i++) {
                        if (angle_compensate_nodes.getDistance(i) / 4.0f / 1000 > 0 && angle_compensate_nodes.getDistance(i) / 4.0f / 1000 < 8) {
                            
                            
                            
                            
                            compensate_nodes[compensate_count++] = angle_compensate_nodes[i]; //
                        }
                    }

                    // veröffentliche nur die korrigierten Scan-Daten
                   
                    publish_scan(compensate_nodes, compensate_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                
                } else {

                    int ANGLE_MIN = RAD2DEG(angle_min);
                    int ANGLE_MAX = RAD2DEG(angle_max);
                    const int MAX_NODES = 8192;
                    
                    sl_lidar_response_measurement_node_hq_t filtered_nodes[MAX_NODES];
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
                        const float distance = nodes[i].dist_mm_q2 / 4.0f / 1000;
                        if(distance > 0 && distance < 8) {


                            filtered_nodes[filtered_count++] = nodes[i];
                       
                       
                        }
                    }
                    publish_scan(filtered_nodes, filtered_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                    }
            loop ++  
            } 
        

        
    

    
    
    return 0;
}
