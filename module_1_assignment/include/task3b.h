#ifndef MULTISENSOR_H
#define MULTISENSOR_H

#include <vector>
#include <string>
#include <iostream>
using namespace std;

template <typename T>
class MultiSensor {
private:
    string sensorName;
    vector<T> sensorData;
    string sensorUnit;

public:
    MultiSensor(const string &name, const vector<T> &data, const string &unit);
    void calibrateSensor();
    void readData();
    void processSensorData();
    void decideAction();
};

#endif // MULTISENSOR_H