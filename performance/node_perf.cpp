

 
#include "node_info.h"
#include <iostream>
#include <string>
#include <chrono>

using namespace std;

# define M_PI           3.14159265358979323846

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


#include <chrono>



// Timer zum teseten des Codes (die Zeit die der Code benötigt um die While-Schleife zu beenden)
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



const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

float angle_min = DEG2RAD*(0.0f);
float angle_max = DEG2RAD*361.0f;

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

/*
    for (size_t i = 0; i < node_count; ++i) {
        
        cout << "Angle = " << RAD2DEG*nodes.getAngle(i) << "    Distance = " << nodes.getDistance(i) << endl; 
    }
*/    

}


static float getAngle(double node)
{
    return RAD2DEG * node  ;
}


int main() {
    

    {

    Timer timer;



    // simulierung der Daten 
    const int size_data = 10;

    NodeInfo nodes(size_data); 
    
    double distances[size_data] = {5.0, 1.0, 1.0, 0.0, 8.0, 2.0 , 4.0, 1 , 3.0 , 1.0};
    nodes.setDistance(distances);
    
    double angles[size_data] = {0, M_PI/10 , M_PI/8 , M_PI/6 , M_PI/5 , M_PI/4 , M_PI/2, M_PI , 1.5*M_PI, 2*M_PI};
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
    float angle_compensate_multiple = 1.0;
    int points_per_circle = 360;
    std::string scan_mode;
    float max_distance;
    double scan_frequency;
    
    angle_compensate_multiple = points_per_circle/360.0  + 1;

    
    angle_compensate = true; // auswahl zum Testen der zwei Schleifen 

    

    double scan_duration;
    int loop = 0;
    
    while (loop < 100000) {   //min 100000 um einen konstanten wert für Elipse Time zu erhalten 
       
        size_t   count = size_data;

        
        
        double start_scan_time = 0;
        double end_scan_time = 100;
        scan_duration = (end_scan_time - start_scan_time);
        

        int ANGLE_MIN = RAD2DEG*angle_min;
        int ANGLE_MAX = RAD2DEG*angle_max;


                if (angle_compensate) {
                    

                    int filtered_count = 0;
                    NodeInfo filtered_nodes(size_data);

                    for (int i = 0; i < count; i++) {
                        if (getAngle(nodes.getAngle(i)) >= ANGLE_MIN && getAngle(nodes.getAngle(i)) <= ANGLE_MAX) {
                            
                            
                            filtered_nodes.setAngle(filtered_count, nodes.getAngle(i));
                            filtered_nodes.setDistance(filtered_count++, nodes.getDistance(i));
                            
                            
                        }
                    }


                    // Winkelkorrektur auf gefilterte Scan-Daten anwenden

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

                    int compensate_count = 0;
                    NodeInfo compensate_nodes(size_data);

                    for (int i = 0; i < angle_compensate_nodes_count; i++) {
                        if (angle_compensate_nodes.getDistance(i)  > 0 && angle_compensate_nodes.getDistance(i) < 800) {
                                                        
                            
                            
                            compensate_nodes.setDistance(compensate_count, angle_compensate_nodes.getDistance(i));
                            compensate_nodes.setAngle(compensate_count++, angle_compensate_nodes.getAngle(i)); 

                        }
                    }

                    // veröffentliche nur die korrigierten Scan-Daten
                   
                    publish_scan(compensate_nodes, compensate_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                
                
                } else {

                    
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

                    
                    for(int i = start_node; i <= end_node; i++) {
                        float distance = nodes.getDistance(i) ;
                        if(distance > 0 && distance < 8) {


                            filtered_nodes.setAngle(filtered_count, nodes.getAngle(i));
                            filtered_nodes.setDistance(filtered_count++, nodes.getDistance(i));

                       
                       
                        }
                    }
                    publish_scan(filtered_nodes, filtered_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance);
                    }
            loop ++  ;



            } 
        

        
    

    
    }
    return 0;
}
