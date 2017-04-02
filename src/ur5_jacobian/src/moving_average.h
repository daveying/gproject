#include "vector"
#include "ros/ros.h"
#include <Eigen/Dense> 
using namespace std;
class MovingAverage
{
public:
    MovingAverage(int length);
    ~MovingAverage(){};
    
    void in(Eigen::MatrixXd item);
    Eigen::MatrixXd out();
private:
    int len;
    vector<Eigen::MatrixXd> container;
    int index;
};

