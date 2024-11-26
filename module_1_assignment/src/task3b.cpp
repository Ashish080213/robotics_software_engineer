#include "task3b.h"

template <typename T>
MultiSensor<T>::MultiSensor(const string &name, const vector<T> &data, const string &unit)
    : sensorName(name), sensorData(data), sensorUnit(unit) {
    cout << "Robot created with " << sensorName << " Sensor." << endl;
}

template <typename T>
void MultiSensor<T>::calibrateSensor() {
    cout << "Calibrating " << sensorName << " Sensor..." << endl;
}

template <typename T>
void MultiSensor<T>::readData() {
    cout << "Processing " << sensorName << " sensor data..." << endl;
    for (size_t i = 0; i < sensorData.size(); ++i) {
        cout << sensorName << " sensor value at " << i << " sec is " << sensorData[i] << sensorUnit << endl;
    }
}

template <typename T>
void MultiSensor<T>::processSensorData() {
    cout << "Applying PID to Sensor values..." << endl;
}

template <typename T>
void MultiSensor<T>::decideAction() {
    cout << "Deciding action based on sensor data..." << endl;
}
