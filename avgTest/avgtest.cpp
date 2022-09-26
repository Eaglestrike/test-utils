//models limlight odom averaging
#include <iostream>
#include <vector>
using namespace std;

vector<pair<double, double>> vision;

int main() {
    //read in test odom & vision data
    vision.resize(100);
    freopen("vision.txt", "r", stdin);
    for (int i = 0; i < 100; i++) {
        double a, b;
        cin >> a >> b;
        vision[i] = {a, b};
    }

    freopen("avg.txt", "w", stdout);

    //simulate running through 200 period cycles
    //testing where odom attempts to follow y=x, so add t each time while actual limelight is y=sqrt(x)
    double rx, ry, lx, ly;
    rx = 0; ry = 0; lx = 0; ly = 0;
    for (int i = 0; i < 200; i++) {
        rx += 0.02;
        ry += 0.02;
        if (i % 2 == 0) {
            lx = vision[i/2].first;
            ly = vision[i/2].second;
        }

        //take weighted average of limelight & wheel with 4% limelight
        // rx = 0.04*lx + 0.96*rx;
        // ry = 0.04*ly + 0.96*ry;

        double dX = lx - rx;
        double dY = ly - ry;
        rx += dX * 0.05;
        ry += dY * 0.05;

        cout << rx << "\t" << ry << "\n";
    }

}