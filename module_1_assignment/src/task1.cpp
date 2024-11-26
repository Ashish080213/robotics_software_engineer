#include <iostream>
#include <vector>
#include <string>

using namespace std;

class RobotClass
{
public:
    RobotClass(const string& robotName, float robotSpeed, float robotWeight, const string& robotSize, int sensors)
        : name(robotName), speed(robotSpeed), weight(robotWeight), size(robotSize), no_of_sensors(sensors) {
        cout << "Robot " << name << " Initialized" << endl;
        Rname();
        Rspeed();
        Rweight();
        Rsize();
        Rsensors();
        cout << endl;
    }
    void Rname() { cout << "Robot name is " << name << endl; }

    void Rspeed() { cout << "Robot " << name << " speed is " << speed << "m/s" << endl; }

    void Rweight() { cout << "Robot " << name << " weight is " << weight << "g" << endl; }

    void Rsize() { cout << "Robot " << name << " size is " << size << endl; }

    void Rsensors() { cout << "Robot " << name << " has " << no_of_sensors << " sensors." << endl; }

    void moveForward(float speed) { cout << "Robot " << name << " moving forward by " << speed << "m/s." << endl; }

    void moveBackward(float speed) { cout << "Robot " << name << " moving backward by " << speed << "m/s." << endl; }

    void stop() { cout << "Robot " << name << " stopped." << endl; }

private:
    string name;
    float speed;
    float weight;
    string size;
    int no_of_sensors;
};

namespace RobotType1 {
    RobotClass r1("R1", 2.0, 1000.0, "30cmx30cmx10cm", 3);
}

namespace RobotType2 {
    RobotClass r2("R2", 3.0, 2000.0, "40cmx30cmx10cm", 6);
}

int main()
{

    RobotType1::r1.moveForward(2.0);
    RobotType2::r2.moveForward(3.0);
    RobotType1::r1.moveBackward(2.0);
    RobotType2::r2.moveBackward(3.0);
    RobotType1::r1.stop();
    RobotType2::r2.stop();

    return 0;
}