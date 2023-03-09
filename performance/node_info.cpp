#include "node_info.h"
#include <cstring>

NodeInfo::NodeInfo(int size) : size_(size) {
    memset(distance_, 0, size_ * sizeof(double));
    memset(angle_, 0, size_ * sizeof(double));
}

void NodeInfo::setDistance(double distances[]) {
    memcpy(distance_, distances, size_ * sizeof(double));
}

void NodeInfo::setAngle(double angles[]) {
    memcpy(angle_, angles, size_ * sizeof(double));
}

double* NodeInfo::getDistance() {
    return distance_;
}

double* NodeInfo::getAngle() {
    return angle_;
}