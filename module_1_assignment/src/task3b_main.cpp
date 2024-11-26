#include "task3b.cpp"
#include <iostream>
#include <vector>

using namespace std;

int main() {
    vector<string> temperatureSensorData = {"Cool", "Normal", "Hot"};

    vector<float> distanceSensorData = {100.0, 80.2, 60.8};

    vector<char> robotMovementData = {'R', 'L', 'R'};

    MultiSensor robot1("T1", temperatureSensorData, "°C");

    robot1.calibrateSensor();
    robot1.readData();
    robot1.processSensorData();
    robot1.decideAction();

    cout << endl;

    MultiSensor robot2("D1", distanceSensorData, "cm");

    robot2.calibrateSensor();
    robot2.readData();
    robot2.processSensorData();
    robot2.decideAction();

    cout << endl;

    MultiSensor robot3("M1", robotMovementData, " Direction");

    robot3.calibrateSensor();
    robot3.readData();
    robot3.processSensorData();
    robot3.decideAction();

    return 0;
}