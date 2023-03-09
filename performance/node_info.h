#ifndef NODE_INFO_H
#define NODE_INFO_H

class NodeInfo {
  private:
    double distance_[8192];
    double angle_[8192];
    int size_;
  
  public:
    NodeInfo(int size = 1);
    void setDistance(double distances[]);
    void setAngle(double angles[]);
    double* getDistance();
    double* getAngle();
};

#endif