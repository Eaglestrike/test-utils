//models limlight odom averaging
#include <iostream>
#include <vector>
using namespace std;

vector<pair<double, double>> odom;
vector<pair<double, double>> vision;

int main() {
    //read in test odom & vision data
    odom.resize(200);
    vision.resize(100);
    freopen("odom.txt", "r", stdin);
    for (int i = 0; i < 200; i++) {
        double a, b;
        cin >> a >> b;
        odom[i] = {a, b};
    }
    freopen("vision.txt", "r", stdin);
    for (int i = 0; i < 100; i++) {
        double a, b;
        cin >> a >> b;
        vision[i] = {a, b};
    }

    freopen("avg.txt", "w", stdout);

    //simulate running through 200 period cycles
    double rx, ry, lx, ly;
    for (int i = 0; i < 200; i++) {
        rx = odom[i].first;
        ry = odom[i].second;
        if (i % 2 == 0) {
            lx = vision[i/2].first;
            ly = vision[i/2].second;
        }

        double dX = lx - rx;
        double dY = ly - rx;
        rx += dX * 0.05;
        ry += dY * 0.05;


        cout << rx << "\t" << ry << "\n";
    }

}