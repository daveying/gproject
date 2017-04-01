#include "iostream"
#include "moving_average.h"

using namespace std;

MovingAverage::MovingAverage(int length)
{
    if(length <= 0) throw;
    this->len = length;
    index = 0;
}

void MovingAverage::in(double item)
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

double MovingAverage::out()
{
    int length = container.size();
    double sum = 0.0;
    for(int i = 0; i < length; i++)
    {
        sum += container[i];
    }
    return sum / length;
}

/*
int main(int argc, char **argv)
{
    MovingAverage test(4);
    test.in(5);
    cout << test.out() << endl;
    test.in(4);
    cout << test.out() << endl;
    test.in(4);
    cout << test.out() << endl;
    test.in(4);
    cout << test.out() << endl;
    test.in(4);
    cout << test.out() << endl;
}
*/
