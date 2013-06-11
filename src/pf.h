#ifndef PF_H
#define PF_H

#include <cmath>
#include <vector>
#include "quaternion.h"
using namespace std;

class PFRobot {
public:
    PFRobot();
private:
    struct particle {
        // internal state
        double x, y, z, dx, dy, dz;
        quaternion q, dq;
        double weight;
    };
    struct command {
        // amount of thrust to apply to each of the 4 rotors
        double f1, f2, f3, f4;
    };
    struct reading {
        // sensor reading
    };
    const int MIN_PARTICLES = 1000; // minimum number of particles to generate on each iteration
    const double GRAVITY = -9.806; // gravitational accelerationm/s/s
    void particleFilter(reading s, command c);
    void motionUpdate(command c, particle * p);
    void sensorUpdate(reading s, particle * p);
    vector <particle> bel(MIN_PARTICLES);
};
#endif
