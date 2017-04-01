#include "vector"
using namespace std;

class MovingAverage
{
public:
    MovingAverage(int length);
    ~MovingAverage(){};
    
    void in(double item);
    double out();
private:
    int len;
    vector<double> container;
    int index;
};

