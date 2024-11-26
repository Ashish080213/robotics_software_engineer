#include "task3.h"

TemperatureSensor::TemperatureSensor(string name, const vector<int> &data)
    : sensorName(name), sensorData(data) { cout << "Robot created with " << sensorName << " Temperature Sensor." << endl; }

void TemperatureSensor::calibrateSensor() { cout << "Calibrating Temperature Sensor..." << endl; }

void TemperatureSensor::readData()
{
  cout << "Processing " << sensorName << " sensor data....." << endl;
  for (int i = 0; i < sensorData.size(); i++)
  {
    cout << sensorName << " sensor value at " << i << " sec is " << sensorData[i] << "°C" << endl;
  }
}

void TemperatureSensor::processSensorData() { cout << "Applying PID to Sensor values.." << endl; }

void TemperatureSensor::decideAction() { cout << "Deciding action based on sensor data..." << endl; }


DistanceSensor::DistanceSensor(string name, const vector<int> &data)
    : sensorName(name), sensorData(data) { cout << "Robot created with " << sensorName << " Distance Sensor." << endl; }

void DistanceSensor::calibrateSensor() { cout << "Calibrating Distance Sensor..." << endl; }

void DistanceSensor::readData()
{
  cout << "Processing " << sensorName << " Distance sensor data....." << endl;
  for (int i = 0; i < sensorData.size(); i++)
  {
    cout << sensorName << " sensor value at " << i << " sec is " << sensorData[i] << "cm" << endl;
  }
}

void DistanceSensor::processSensorData() { cout << "Applying PID to Sensor values.." << endl; }

void DistanceSensor::decideAction() { cout << "Deciding action based on sensor data..." << endl; }