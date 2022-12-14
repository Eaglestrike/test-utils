//models odom reading as y = t and vision as y = sqrt(t)
//odom timestep is t = 0.02, limelight timestep 0.04
//goes from t = 0 to t = 4

#include <iostream>
#include <math.h>
using namespace std;

int main(){
    freopen("vision.txt", "w", stdout);
    for (double t = 0; t <= 4; t+= 0.04) {
        cout << t << "\n" << sqrt(t) + 0.05*sin(50*t) + 0.05*cos(10*t) << "\n";
    }

    return 0;
}