#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <vector>
#include <string>
#include <iostream>
using namespace std;

class TemperatureSensor {
    private:
        string sensorName;
        vector<int> sensorData;

    public:
        TemperatureSensor(string name, const vector<int> &data);
        void calibrateSensor();
        void readData();
        void processSensorData();
        void decideAction();
};
#endif // TEMPERATURESENSOR_H

class DistanceSensor {
    private:
        string sensorName; 
        vector<int> sensorData;

    public:
        DistanceSensor(string name, const vector<int> &data);
        void calibrateSensor();
        void readData();
        void processSensorData();
        void decideAction();
};
#endif // DISTANCESENSOR_H