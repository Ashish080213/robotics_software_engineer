#include "task3.h"
#include <iostream>
#include <vector>

using namespace std;

int main() {
    vector<int> temperatureSensorData = {10, 20, 30, 40, 50};

    vector<int> distanceSensorData = {100, 80, 60, 40, 20};

    TemperatureSensor robot1("T1", temperatureSensorData);

    robot1.calibrateSensor();
    robot1.readData();
    robot1.processSensorData();
    robot1.decideAction();

    cout << endl;

    DistanceSensor robot2("D1", distanceSensorData);

    robot2.calibrateSensor();
    robot2.readData();
    robot2.processSensorData();
    robot2.decideAction();

    return 0;
}