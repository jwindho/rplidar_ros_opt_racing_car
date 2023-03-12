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
