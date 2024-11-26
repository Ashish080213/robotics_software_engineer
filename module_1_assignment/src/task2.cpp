#include <iostream>
#include <vector>
#include <memory>
using namespace std;

class Robot {
public:
    template <typename N, typename D, typename U>
    void processData(N name, D& data, U unit) {

        int n = (*data).size();

        cout << "Processing " << name << " sensor data....." << endl;
        for (int i = 0; i < n; i ++){
            cout << name << " sensor value at " << i << " sec is " << (*data)[i] << unit << endl;
        }

    }
};

int main() {
    Robot myRobot;

    // vector<int> TemperatureSensor = {10, 20, 30, 40, 50};
    auto TemperatureSensor = make_unique<vector<int>>(initializer_list<int>{10, 20, 30, 40, 50});

    // vector<int> DistanceSensor = {100, 80, 60, 40, 20};
    auto DistanceSensor = make_unique<vector<int>>(initializer_list<int>{100, 80, 60, 40, 20});


    myRobot.processData("Temperature", TemperatureSensor, "°C");
    cout << endl;
    myRobot.processData("Distance", DistanceSensor, "cm");
    return 0;
}
