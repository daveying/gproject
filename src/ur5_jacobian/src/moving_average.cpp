#include "iostream"
#include "moving_average.h"

using namespace std;

MovingAverage::MovingAverage(int length)
{
    if(length <= 0) throw;
    this->len = length;
    index = 0;
}

void MovingAverage::in(Eigen::MatrixXd item)
{
    if(container.size() < len)
    {
        container.push_back(item);
    }
    else
    {
        container[index] = item;
        index++;
        if(index == len) index = 0;
    }
}

Eigen::MatrixXd MovingAverage::out()
{
    int length = container.size();
    if(length == 0)throw;
    Eigen::MatrixXd sum(container[0].rows(), container[0].cols());
    for(int i = 0; i < length; i++)
    {
        sum += container[i];
    }
    return sum / length;
}


int main(int argc, char **argv)
{
    Eigen::MatrixXd a(2, 1);
    a(0,0) = 5;
    MovingAverage test(4);
    test.in(a);
    cout << test.out() << endl;
    a(0,0) = 4;
    test.in(a);
    cout << test.out() << endl;
    test.in(a);
    cout << test.out() << endl;
    test.in(a);
    cout << test.out() << endl;
    test.in(a);
    cout << test.out() << endl;
}

